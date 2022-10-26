# Odometry calculation for AgileX Robotics SCOUT 2.0
Project for the Robotics course 2020/2021, offered by Politecnico di Milano.
The goal was to compute odometry using approximate skid steering kinematics, given the RPM of each motor of the robot.
The project is written in **ROS(C++)**, an open-source collection of robotics frameworks.


Robot - AgileX Robotics SCOUT 2.0 ([global.agilex.ai/products/scout-2-0](https://global.agilex.ai/products/scout-2-0))
---
* All-round unmanned ground vehicle for industrial applications
* Skid steering
* 4 motors

File structure
---
* **/bags**

   Files containing recorded data from the AgileX Scout robot: bag1.bag, bag2.bag, bag3.bag.
* **/cfg**

   File containing the definition of the dynamicly reconfigurable integration method (Euler or Runge-Kutta) in python: parameters.cfg.
* **/launch**

   File with which you can run the desired bag, set the needed parameters, run the nodes, and prepare the environment: lab.launch.
* **/msg**

   File containing the definitions of the custom messages for the motor speed and the custom odometry: MotorSpeed.msg and OdometryCustom.msg.
* **/src**

   Files containing the source code of the nodes for parsing the pose of the robot, and calculating the odometry: pose_parser.cpp and odometry_calculation.cpp.
* **/srv**

   File containing the definition of the service used to reset the odometry to a given pose: ResetOdometry.srv.

How to start the nodes
---
To run, clone the project, build, and run with **"roslaunch robotics_hw1 lab.launch"**.
The launch file starts the pose_parser and odometry_calculation nodes, sets up the parameters.

To launch with a bag, use **"roslaunch robotics_hw1 lab_with_bag.launch"**
