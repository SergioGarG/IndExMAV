# IndExMAV

Indoor Exploration with Micro-Aerial Vehicles for Reconnaissance Missions

IndExMAV is a project that allows low-cost MAVs to perform SLAM in real-scale environments. The system can be implemented over the following hardware platforms:

1. Parrot Bebop Drone.
2. AR Drone 2.0.

The system use information from the monocular camera and the IMU that are implemented in most of the commercial drones. For this reason, the system needs of a monocular VSLAM algorithm to run. Any algorithm could be implemented remapping the topics, but we have used:

1. LSD-SLAM: https://github.com/tum-vision/lsd_slam
2. ORB-SLAM: https://github.com/raulmur/ORB_SLAM

# Prerequisites

1. ROS
We have tested the system in Ubuntu 12.04 with ROS Hydro and in Ubuntu 14.04 with ROS Indigo.

2. Visual SLAM algorithm
Any VSLAM method which brings the 6 DoF drone's position could be used. The system will recognice automatically LSD-SLAM and ORB-SLAM.

3. Eigen
The project uses Eigen for the calculations.

http://eigen.tuxfamily.org/index.php?title=Main_Page

# Installation

Just go to your workspace project, type catkin_make and press enter on your keyboard.

# Usage

1. Set the desired path for your drone in pid_main_retardos.cpp. 
2. Launch your VSLAM algorithm.
3. Launch the EKF node.
rosrun EKF EKF
4. Make your MAV to take off and launch the PID controller node.
rosrun PID pid_main_retardos

# Related papers:

Indoor SLAM for Micro Aerial Vehicles Control using Monocular Camera and Sensor Fusion, S. García, M. E. López, ICARSC'16
'16
