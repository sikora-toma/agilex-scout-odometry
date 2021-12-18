#include "ros/ros.h"
#include "math.h"
#include "string.h"
#include "tf2/LinearMath/Quaternion.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "nav_msgs/Odometry.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "message_filters/subscriber.h"
#include "dynamic_reconfigure/server.h"
#include "robotics_hw1/parametersConfig.h"
#include "robotics_hw1/OdometryCustom.h"
#include "robotics_hw1/ResetOdometry.h"

// measurements in cm
#define WHEEL_RADIUS 0.1575
#define REAL_BASELINE 0.583
#define APPARENT_BASELINE  1.015 // >0.583 TODO: calibrate

class odometry_calculator;
	odometry_calculator* my_odometry_calculator_global;


class odometry_calculator{
private:
	ros::NodeHandle n; 
	ros::Publisher odometry, odometry_and_type;
	tf2_ros::TransformBroadcaster br;
	ros::Subscriber sub;
	bool method; // euler true runge_kutta false
	double x, y, theta, t;

public:
	odometry_calculator(){
		if(!n.getParam("x0", x))		ROS_INFO("Failed to read paramater x.");
		if(!n.getParam("y0", y))		ROS_INFO("Failed to read paramater y.");
		if(!n.getParam("theta0", theta))ROS_INFO("Failed to read paramater theta.");

		ROS_INFO("Loaded parameters.");

		odometry = n.advertise<nav_msgs::Odometry>("odometry_calculated", 1000);
		odometry_and_type = n.advertise<robotics_hw1::OdometryCustom>("odometry_and_type", 1000);
    	sub = n.subscribe("twistStamped", 1000, &odometry_calculator::callback, this);

		method = true;
	}
	void callback(const geometry_msgs::TwistStampedConstPtr& twistStamped) {
		//ROS_INFO("Twist stamped velocity: %f, rotational speed %f", twistStamped->twist.linear.x, twistStamped->twist.angular.z);

		nav_msgs::Odometry odometry_calculated;
		odometry_calculated.header.stamp = ros::Time::now();
		odometry_calculated.header.frame_id = "world";
		odometry_calculated.child_frame_id = "odometry_calculated";

		if(method)	euler_odometry(twistStamped->twist.linear.x, twistStamped->twist.angular.z, twistStamped->header.stamp.toSec()-t);
		else		runge_kutta_odometry(twistStamped->twist.linear.x, twistStamped->twist.angular.z, twistStamped->header.stamp.toSec()-t);

		// TODO: Update update values		
		odometry_calculated.pose.pose.position.x = x;
		odometry_calculated.pose.pose.position.y = y;

		tf2::Quaternion myQuaternion;
		myQuaternion.setRPY(0, 0, theta);
		odometry_calculated.pose.pose.orientation.x = myQuaternion.getX();
		odometry_calculated.pose.pose.orientation.y = myQuaternion.getY();
		odometry_calculated.pose.pose.orientation.z = myQuaternion.getZ();
		odometry_calculated.pose.pose.orientation.w = myQuaternion.getW();

		odometry.publish(odometry_calculated);

        t = twistStamped->header.stamp.toSec();

		robotics_hw1::OdometryCustom my_custom_odometry_message;
		my_custom_odometry_message.method = method?"euler":"rk";
		my_custom_odometry_message.odom = odometry_calculated;

		odometry_and_type.publish(my_custom_odometry_message);

		geometry_msgs::TransformStamped transformStamped;
		transformStamped.header.stamp = ros::Time::now();
		transformStamped.header.frame_id = "world";
		transformStamped.child_frame_id = "odometry_calculated";
		transformStamped.transform.translation.x = x;
		transformStamped.transform.translation.y = y;
		transformStamped.transform.translation.z = 0.0;
		transformStamped.transform.rotation.x = myQuaternion.x();
		transformStamped.transform.rotation.y = myQuaternion.y();
		transformStamped.transform.rotation.z = myQuaternion.z();
		transformStamped.transform.rotation.w = myQuaternion.w();
		br.sendTransform(transformStamped);
	}

	void euler_odometry(double linear_velocity, double angular_velocity, double delta_time){
		x += linear_velocity*delta_time*cos(theta);
		y += linear_velocity*delta_time*sin(theta);
		theta += angular_velocity*delta_time;
	}

	void runge_kutta_odometry(double linear_velocity, double angular_velocity, double delta_time){
		x += linear_velocity*delta_time*cos(theta+angular_velocity*delta_time/2);
		y += linear_velocity*delta_time*sin(theta+angular_velocity*delta_time/2);
		theta += angular_velocity*delta_time;
	}
	void set_x(double newX){			x=newX;				}
	void set_y(double newY){			y=newY;				}
	void set_theta(double newTheta){	theta=newTheta;		}
	void set_method(bool newMethod){	method=newMethod;	}

	double get_x(){						return x;			}
	double get_y(){						return y;			}
	double get_theta(){					return theta;		}
	bool get_method(){					return method;		}
};



bool reset_odometry(std_srvs::Empty::Request, std_srvs::Empty::Response){
	ROS_INFO("Reset Odom To Zero Request");
	my_odometry_calculator_global->set_x(0);
	my_odometry_calculator_global->set_y(0);
	my_odometry_calculator_global->set_theta(0);

	return true;
}
bool reset_odometry_to_given_pose(robotics_hw1::ResetOdometry::Request  &req, robotics_hw1::ResetOdometry::Response &res){
	ROS_INFO("Reset Odom To Pose Request: %f %f %f",
	req.x0, req.y0, req.theta0);
	my_odometry_calculator_global->set_x(req.x0);
	my_odometry_calculator_global->set_y(req.y0);
	my_odometry_calculator_global->set_theta(req.theta0);

	return true;
}


void method_callback(robotics_hw1::parametersConfig &config, bool value) {
	ROS_INFO("Reconfigure Request: %s",
	config.method?"Euler":"Runge-Kutta");
	my_odometry_calculator_global->set_method(config.method);
}


int main(int argc, char **argv){
  	ROS_INFO("Started odometry calculation module.");

	ros::init(argc, argv, "odometry_calculation");

	ros::NodeHandle n;
	//odometry_calculator* odom_calc = new odometry_calculator();
	my_odometry_calculator_global = new odometry_calculator();

    ros::ServiceServer service1 = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
						("reset_odometry", boost::bind(&reset_odometry, _1, _2));
    ros::ServiceServer service2 = n.advertiseService<robotics_hw1::ResetOdometry::Request, robotics_hw1::ResetOdometry::Response>
						("reset_odometry_to_given_pose", boost::bind(&reset_odometry_to_given_pose, _1, _2));

	ROS_INFO("Ready to reset the odometry.");



	dynamic_reconfigure::Server<robotics_hw1::parametersConfig> server;
	dynamic_reconfigure::Server<robotics_hw1::parametersConfig>::CallbackType f = boost::bind(&method_callback, _1, _2);
	server.setCallback(f); 

  	ros::spin();

  return 0;
}


