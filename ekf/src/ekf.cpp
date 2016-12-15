// Extended Kalman Filter with delay correction

#include "predictor.h"
#include "correccion_vslam.h"
#include "correccion_imu.h"
#include "escala.h"
#include "variables_retardos.h"
using namespace std;

/* Se normalizan todos los ejes a:
   x : desplazamiento de profundidad (adelante y atrás)
   y : desplazamiento lateral (izquierda y derecha)
   z : desplazamiento vertical (arriba y abajo)

   Negativo |   | Positivo
   ---------|---|----------
    Atrás   | X | Adelante
    Derecha | Y | Izquierda
    Abajo   | Z | Arriba
 */

/////////////////////////////////////MAIN//////////////////////////////////////

int main(int argc, char **argv)
{
	ros::init(argc, argv, "ekf");

	ros::NodeHandle n;

	//Suscriptores
	ros::Subscriber sub_poseLSD = n.subscribe("lsd_slam/pose", 1000, CallbackPoseLSD); //El nodo al que se subscribe en caso de ser LSD

	ros::Subscriber sub_IMU_ardrone = n.subscribe("ardrone/navdata", 1000, CallbackIMUArdrone); //El nodo al que se subscribe para obtener datos de la IMU en caso de ser ardrone con legacy ON

	ros::Subscriber sub_altitude_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/AltitudeChanged", 1000, CallbackAltitudeBebop); //Nodo bebop altura
	ros::Subscriber sub_attitude_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/AttitudeChanged", 1000, CallbackAttitudeBebop); //Nodo bebop ángulos
	ros::Subscriber sub_speed_bebop = n.subscribe("bebop/states/ARDrone3/PilotingState/SpeedChanged", 1000, CallbackVelocidadBebop); //Nodo bebop velocidades

	ros::Subscriber sub_cmd = n.subscribe("cmd_vel", 1000, CallbackCMD); //El nodo al que se subscribe para conseguir los comandos enviados al dron
	ros::Subscriber sub_cmd_bebop = n.subscribe("bebop/cmd_vel", 1000, CallbackCMDBebop); //El nodo al que se subscribe para conseguir los comandos enviados al bebop

	tf::TransformListener listener;
	//tf::TransformListener listener_tf(ros::Duration(10)); //Descomentar para usar el cambio de frame

	//Objetos de las clases
	predictor prediccion;
	corrector_vslam correctorvslam;
	corrector_imu correctorimu;
	escala_obj escala_cal;

	//Publicadores
	ros::Publisher chatter_pub = n.advertise<ekf::mensaje_kalman_array>("kalman_topic", 1000);
	ros::Publisher chatter_pub_prediccion = n.advertise<ekf::mensaje_kalman_array>("kalman_topic_retardos", 1000);


	//Defino variables de tiempo
	ros::Time current_time;

	ros::Rate loop_rate(25); //valor en Hz del rate

	while (ros::ok())
	{
		ros::spinOnce();
		current_time = ros::Time::now();

		ekf::mensaje_kalman data;
		ekf::mensaje_kalman_array msg_pub;

		ekf::mensaje_kalman data_prediccion;
		ekf::mensaje_kalman_array msg_pub_prediccion;

		tf::StampedTransform transform;

		are_equal=false;
		are_equal_0=false;

		if(inicializacion==true)
		{
			init();
			inicializacion=false;
		}

		//Se lee la posición con 6DoF según ORB
		if(!tecnica_vslam)
		{
			try
			{
				listener.lookupTransform("/ORB_SLAM/World", "/ORB_SLAM/Camera",
						ros::Time(0), transform);
			}

			catch (tf::TransformException &ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
				continue;
			}

			pose_vslam[1]=-transform.getOrigin().x();
			pose_vslam[2]=-transform.getOrigin().y();
			pose_vslam[0]=transform.getOrigin().z();
			orientacion_vslam[0]=transform.getRotation().x();
			orientacion_vslam[1]=transform.getRotation().y();
			orientacion_vslam[2]=transform.getRotation().z();
			orientacion_vslam[3]=transform.getRotation().w();
		}

		if(z_imu[2]<0) z_imu[2]=-z_imu[2];
		if(pose_vslam[2]<0) pose_vslam[2]=-pose_vslam[2];

		//Se calcula la estimación de VSLAM con escala real
		escala_cal.Calcula_escala(pose_vslam, orientacion_vslam, z_imu[2], posicion_anterior);

		for(i=0;i<3;i++)
		{
			z_vslam[i]=escala_cal.posicion_real[i];
			z_vslam[i+3]=escala_cal.orientacion_real[i];
		}

		//Se actualiza el array que contiene el buffer de los comandos u
		for(i=0;i<N+M;i++)
		{
			for(j=0;j<4;j++)
			{
				if(i==N+M-1)
				{
					u_buffer[j][i]=u[j];
				}
				else
				{
					u_buffer[j][i]=u_buffer[j][i+1];
				}
			}
		}

		//Aquí se comprueba si el feed de vídeo del drone se ha congelado viendo si las últimas 6 estimaciones de posición son iguales

		for(i=0;i<F;i++)
		{

			for(j=0;j<3;j++)
			{

				if(i==0)
				{
					if (pose_vslam[j] == posicion_anterior_vslam[i][j]) are_equal_0 = true;
				}
				else if(i!=0 && are_equal_0==true)
				{
					if(posicion_anterior_vslam[i][j]==posicion_anterior_vslam[i-1][j])
					{
						are_equal_0=true;
						if(i==(F-1)) are_equal=true;
					}
					else
					{
						are_equal_0=false;
						break;
					}
				}
			}
		}

		//Aquí se aplican los 3 modelos del EKF. Si se ha detectado que el feed de vídeo se ha congelado, se deja de utilizar el modelo
		//de corrección de VSLAM.

		//Si el feed está bloqueado
		if(are_equal==true)
		{
			for(i=0;i<10;i++)
			{

				x_actual[i]=x[i][0];		
				for(j=0;j<10;j++)
				{
					covarianza_x_actual[j][i]=covarianza_x[j][i][0];
				}
			}

			//Se hace la corrección
			correctorimu.corregir_imu(x_actual,covarianza_x_actual, z_imu);

			//Se hace la predicción de las siguientes M muestras
			for(i=M;i>0;i--)
			{
				if(i==M)
				{
					for(j=0;j<10;j++)
					{
						x_actual[j]=correctorimu.estado_correccion_imu[j];
						for(l=0; l<10; l++)
						{
							covarianza_x_actual[l][j]=correctorimu.varianza_correccion_imu[l][j];
						}		
					}				
				}

				else
				{
					for(j=0;j<10;j++)
					{
						x_actual[j]=x[j][M-(i+1)];
						for(l=0;l<10;l++)
						{
							covarianza_x_actual[l][j]=covarianza_x[l][j][M-(i+1)];
						}
					}
				}

				for(j=0; j<4; j++)
				{
					u_actual[j]=u_buffer[j][M-i];
				}

				prediccion.predecir(x_actual, covarianza_x_actual, u_actual, drone_utilizado);

				for(j=0;j<10;j++)
				{
					x[j][M-i]=prediccion.estado_predictor[j];
					for(l=0; l<10; l++)
					{
						covarianza_x[l][j][M-i]=prediccion.varianza_predictor[l][j];
					}		
				}

			}

			//Se almacena la última predicción para enviarla por topic al PID

			for(j=0;j<10;j++)
			{
				x_envio[j]=x[j][M-1];
			}
			//Prueba, se toman como correctas las medidas de la IMU para rotaciones y velocidades
//			x_envio[2]=z_imu[2];
//			x_envio[3]=z_imu[0];
//			x_envio[4]=z_imu[1];
//			x_envio[6]=z_imu[3];
//			x_envio[7]=z_imu[4];
//			x_envio[8]=z_imu[5];

			cout<<"["<<current_time<<"]"<<" Video stream is frozen, estimation discarding VSLAM correction"<<endl;
		}

		else //Si el feed no está bloqueado realizo el filtro.
		{
			//			prediccion.predecir(media, covarianza, u, drone_utilizado);
			//			correctorvslam.corregir_vslam(prediccion.estado_predictor, prediccion.varianza_predictor, z_vslam);
			//			correctorimu.corregir_imu(correctorvslam.estado_correccion_vslam, correctorvslam.varianza_correccion_vslam, z_imu);

			for(i=0;i<10;i++)
			{

				x_actual[i]=x[i][0];
				for(j=0;j<10;j++)
				{
					covarianza_x_actual[j][i]=covarianza_x[j][i][0];
				}
			}

			//Hago la corrección de la predicción del instante actual hecha previamente (x(T-N))
			correctorvslam.corregir_vslam(x_actual, covarianza_x_actual, z_vslam);
			correctorimu.corregir_imu(correctorvslam.estado_correccion_vslam, correctorvslam.varianza_correccion_vslam, z_imu);


			//Hago una predicción de las siguientes muestras (N+M)
			for(i=N+M;i>0;i--)
			{
				if(i==(N+M))
				{
					for(j=0;j<10;j++)
					{
						x_actual[j]=correctorimu.estado_correccion_imu[j];
						for(l=0; l<10; l++)
						{
							covarianza_x_actual[l][j]=correctorimu.varianza_correccion_imu[l][j];
						}
					}
				}

				else
				{
					for(j=0;j<10;j++)
					{
						x_actual[j]=x[j][N+M-(i+1)];
						for(l=0;l<10;l++)
						{
							covarianza_x_actual[l][j]=covarianza_x[l][j][N+M-(i+1)];
						}
					}
				}

				for(j=0; j<4; j++)
				{
					u_actual[j]=u_buffer[j][N+M-i];
				}

				prediccion.predecir(x_actual, covarianza_x_actual, u_actual, drone_utilizado);

				for(j=0;j<10;j++)
				{
					x[j][N+M-i]=prediccion.estado_predictor[j];
					for(l=0; l<10; l++)
					{
						covarianza_x[l][j][N+M-i]=prediccion.varianza_predictor[l][j];
					}
				}

			}

			//Se almacena la última predicción para enviarla por topic al PID
			for(j=0;j<10;j++)
			{
				x_envio[j]=x[j][N+M-1];
			}
			//Prueba, se toman como correctas las medidas de la IMU para rotaciones y velocidades
//			x_envio[2]=z_imu[2];
//			x_envio[3]=z_imu[0];
//			x_envio[4]=z_imu[1];
//			x_envio[6]=z_imu[3];
//			x_envio[7]=z_imu[4];
//			x_envio[8]=z_imu[5];
		}

		//Se recarga la posición anterior de vslam para la comparación en la siguiente iteración
		for(i=(F-1);i>(-1);i--)
		{
			for(j=0;j<3;j++)
			{
				if(i==0) posicion_anterior_vslam[i][j]=pose_vslam[j];
				else posicion_anterior_vslam[i][j]=posicion_anterior_vslam[i-1][j];
			}
		}

		//Se crea este bucle para poner la salida deseada y poder hacer pruebas con todos los modelos por separado
		for(i=0;i<10;i++)
		{
			modelo[i]=correctorimu.estado_correccion_imu[i];
			media[i]=modelo[i];
			for(j=0;j<10;j++)
			{
				covarianza_modelo[i][j]=correctorimu.varianza_correccion_imu[i][j];
				covarianza[i][j]=covarianza_modelo[i][j];
			}
		}


		for(i=0;i<3;i++)
		{
			//Se evitan saltos o errores de cálculo frente a irregularidades del terreno mediante este ajuste
			if(abs(abs(media[i])-abs(posicion_anterior[i]))>0.1) media[i]=posicion_anterior[i];
			//Se evitan problemas en el algoritmo eliminando medidas no deseadas
			if(isnan(media[i])) media[i]=0;
			if(isnan(media[i+3])) media[i+3]=0;
			posicion_anterior[i]=media[i];
			orientacion_anterior[i]=media[6+i];
		}

		//Prueba, se toman como correctas las medidas de la IMU para rotaciones y velocidades
//		media[2]=z_imu[2];
//		media[3]=z_imu[0];
//		media[4]=z_imu[1];
//
//		media[6]=z_imu[3];
//		media[7]=z_imu[4];
//		media[8]=z_imu[5];


		//Se elimina el offset inicial de yaw para que inicie siempre en 0
		if((z_imu[2]>=1)) //&& (primera_vez==1))
		{
			Calculo_orientacion(z_imu[5]);
			primera_vez2=1;

			//cout<<"Yaw ajustada a: "<<media[8]<<endl;
		}

		//Se realiza la conversión entre la posición de la cámara y el centro del drone, descomentar para usar la función
		//ConversorCamara_listener(listener_tf);

		x_envio[8]=media[8];
		/////////////Se envían los datos

		for(i=0;i<10;i++)
		{
			data.estado=media[i];
			msg_pub.estado_vector.push_back(data);
		}


		for(i=0;i<4;i++)
		{
			if(i==3)
			{
				data_prediccion.estado=x_envio[8];
			}
			else
			{
				data_prediccion.estado=x_envio[i];
			}
			msg_pub_prediccion.estado_vector.push_back(data_prediccion);
		}

		//Se publica
		chatter_pub.publish(msg_pub);
		chatter_pub_prediccion.publish(msg_pub_prediccion);

		//Fin de bucle
		loop_rate.sleep();

	}

	//ros::spin();

	return 0;
}




