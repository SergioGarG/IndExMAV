#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/message.h"
#include "ros/time.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include "std_msgs/Float64.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tfMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Transform.h"
#include <iostream>
#include <math.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <algorithm>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "ekf/Ardrone3PilotingStateAltitudeChanged.h"
#include "ekf/Ardrone3PilotingStateAttitudeChanged.h"
#include "ekf/Ardrone3PilotingStateSpeedChanged.h"
#include "ekf/Navdata.h"
#include "ekf/mensaje_kalman.h"
#include "ekf/mensaje_kalman_array.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/QuaternionStamped.h"
using namespace std;

//Constantes

const double pi=M_PI;
const int M=3; //Retardo en la señal de control
const int N=4; //Retardo de la imagen
const int F=4; //Número de iteraciones con el mismo frame para detectar parón del stream de vídeo

//Variables
double media[10]={0};
double modelo[10]={0};

double** x = new double*[10];

double covarianza_x[10][10][N+M]={{0}};
double u_buffer[4][N+M]={{0}};
double aux_covarianza[10][10][N+M]={{0}};
double aux_estado[10][N+M]={{0}};

double x_actual[10]={0};
double covarianza_x_actual[10][10]={{0}};
double u_actual[4]={0};

double x_envio[10]={0};

double u[4]={0};
double z_vslam[6]={0};
double z_imu[6]={0};
double altitud=0;
double pose_vslam[3]={0};
double orientacion_vslam[4]={0};
double posicion_anterior[3]={0};
double posicion_previa[3]={0};
double orientacion_anterior[3]={0};

double posicion_anterior_vslam[6][3]={0};

double diferencia=0;
int primera_vez=0;
int primera_vez2=0;

bool are_equal=false;
bool are_equal_0=false;

bool inicializacion=true;

int i, j, l, k;

int drone_utilizado=1; //0->ardrone; 1->bebop
int tecnica_vslam=0; //0->ORB; 1->LSD

double roll, pitch, yaw;

//////////////////////////FUNCIONES/////////////////////////////////////////

void init()
{
	for(i = 0; i < 10; ++i)
	   x[i] = new double[N+M];

	for(i = 0; i < 10; i++)
	{
		for(j=0; j<N+M; j++)
			x[i][j] = 0;
	}
}

void CallbackPoseLSD(const geometry_msgs::PoseStamped::ConstPtr& msg) //Callback de LSD
{
	/*     Negativo |   | Positivo
		   ---------|---|----------
		   Izquierda| X | Derecha
		   Arriba   | Y | Abajo
		   Atrás    | Z | Adelante
		 */
	//Hago que y sea z para ajustarlo a como se tiene en cuenta en el resto de nodos (z eje vertical en vez de profundidad)
	pose_vslam[0] = msg->pose.position.z;
	pose_vslam[1] = msg->pose.position.x;
	pose_vslam[2] = msg->pose.position.y;
	orientacion_vslam[0] = msg->pose.orientation.x;
	orientacion_vslam[1] = msg->pose.orientation.y;
	orientacion_vslam[2] = msg->pose.orientation.z;
	orientacion_vslam[3] = msg->pose.orientation.w;

	pose_vslam[1]=-pose_vslam[1];
	pose_vslam[2]=-pose_vslam[2];
	tecnica_vslam=1;
}

void CallbackIMUArdrone(const ekf::Navdata::ConstPtr& msg)
{

	double roll=0, pitch=0, yaw=0;
	//Se normalizan los resultados: las velocidades se pasan de mm/sg a m/sg, la altura de cm a m y los ángulos de grados a radianes
	z_imu[0]= (msg->vx)/1000;
	z_imu[1]= (msg->vy)/1000;
	z_imu[2] = ((double) msg->altd)/1000;
	z_imu[3] = (double) (msg->rotX)*(3.14/180);
	z_imu[4] = (double) (msg->rotY)*(3.14/180);
	z_imu[5] = (double) (msg->rotZ)*(3.14/180);

	drone_utilizado=0; //Si se utiliza ardrone con legacy ON
}

void CallbackAltitudeBebop(const ekf::Ardrone3PilotingStateAltitudeChanged::ConstPtr& msg)
{
	z_imu[2] = msg->altitude; //Ya viene en metros
	drone_utilizado=1; //Si se utiliza bebop
}

void CallbackAttitudeBebop(const ekf::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg)
{
	double roll=0, pitch=0, yaw=0;

	z_imu[3] = msg->roll; //Ya viene en rad
	z_imu[4] = -msg->pitch; //Ya viene en rad
	z_imu[5] = -msg->yaw; //Ya viene en rad

	if(!primera_vez && z_imu[2]>=1)
	{
		diferencia=z_imu[5];
		cout<<"La diferencia en yaw es: "<<diferencia<<endl;
		primera_vez=1;
	}
}

void CallbackVelocidadBebop(const ekf::Ardrone3PilotingStateSpeedChanged::ConstPtr& msg)
{
	z_imu[0] = msg->speedY; //Ya viene en m/s
	z_imu[1] = msg->speedX; //Ya viene en m/s
	//z_imu[1]=-z_imu[1]; //Se invierte dado que las lecturas del navdata son contrarias a la norma
}

void CallbackCMD(const geometry_msgs::Twist::ConstPtr& msg)
{
	//Se modifica la señal de control para que sea directamente u = vx, vy, vyaw, vz
	u[0]=msg->linear.x; //velocidad en eje x
	u[1]=msg->linear.y; //velocidad en eje y
	u[2]=msg->angular.z; //Yaw
	if(msg->angular.z >= 1) u[2]=1;
	if(msg->angular.z <= -1) u[2]=-1;
	u[3]=msg->linear.z; //Vel Z
}

void CallbackCMDBebop(const geometry_msgs::Twist::ConstPtr& msg)
{
	//Se modifica la señal de control para que sea directamente u = vx, vy, vyaw, vz
	u[0]=msg->linear.x; //velocidad en eje x
	u[1]=msg->linear.y; //velocidad en eje y
	u[2]=msg->angular.z; //Yaw
	u[2]=-u[2]; //Para que el bebop gire positivo->antihorario
	if(msg->angular.z >= 1) u[2]=1;
	if(msg->angular.z <= -1) u[2]=-1;
	u[3]=msg->linear.z; //Vel Z}
}

void Calculo_orientacion(double yaw)
{
	media[8]=yaw-diferencia;
	if(abs(media[8])>pi)
	{
		media[8]=(abs(media[8])-2*pi);
		if(yaw<0) media[8]=-media[8];
	}
}

//void ConversorCamara_listener(const tf::TransformListener& listener) //Descomentar para usar cambio de frame
//{
//  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
//  geometry_msgs::PointStamped camara_point;
//  camara_point.header.frame_id = "base_camara";
//
//  //we'll just use the most recent transform available for our simple example
//  camara_point.header.stamp = ros::Time();
//
//  //just an arbitrary point in space
//  camara_point.point.x = media[0];
//  camara_point.point.y = media[1];
//  camara_point.point.z = media[2];
//
//  try
//  {
//    geometry_msgs::PointStamped base_point;
//    listener.transformPoint("centro_drone", camara_point, base_point);
//
////    ROS_INFO("base_camara: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
////    		camara_point.point.x, camara_point.point.y, camara_point.point.z,
////        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
//
//    media[0]=base_point.point.x;
//    media[1]=base_point.point.y;
//    media[2]=base_point.point.z;
//  }
//  catch(tf::TransformException& ex){
//    ROS_ERROR("Received an exception trying to transform a point from \"base_camara\" to \"base_link\": %s", ex.what());
//  }
//}

//Variables de prueba
double covarianza[10][10]=
		                  {
		                		  {0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				                  {0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0.03, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0.03, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0, 0.08, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08}
		                  };

double covarianza_modelo[10][10]=
		                  {
		                		  {0.1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
				                  {0, 0.1, 0, 0, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0.1, 0, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0.1, 0, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0.1, 0, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0.1, 0, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0.03, 0, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0.03, 0, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0, 0.08, 0},
						          {0, 0, 0, 0, 0, 0, 0, 0, 0, 0.08}
		                  };

