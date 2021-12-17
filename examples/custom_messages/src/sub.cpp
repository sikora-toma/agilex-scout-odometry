#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/TwistStamped.h"
#include "custom_messages/Num.h"
#include "custom_messages/MotorSpeed.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

// measurements in cm
#define WHEEL_RADIUS 0.1575
#define REAL_BASELINE 0.583
#define APPARENT_BASELINE  0.583 // >0.583 TODO: calibrate

#define GEAR_RATIO 38 // 35<x<40 TODO: calibrate
#define PI 3.1415267

void callback(const custom_messages::MotorSpeedConstPtr& fl, 
              const custom_messages::MotorSpeedConstPtr& fr,
			  const custom_messages::MotorSpeedConstPtr& rl,
			  const custom_messages::MotorSpeedConstPtr& rr,
			  const ros::Publisher twistStamped) {
  //ROS_INFO ("Received motor speeds messages: %f, %f, %f, %f", fl->rpm, fr->rpm, rl->rpm, rr->rpm);
	float vl = -(fl->rpm+rl->rpm)/2/GEAR_RATIO/WHEEL_RADIUS/(2*PI)/60, vr = (fr->rpm+rr->rpm)/2/GEAR_RATIO/WHEEL_RADIUS/(2*PI)/60;
	float v = (vl+vr)/2, w = (vl-vr)/2;
	ROS_INFO ("Velocity: %f, rotational speed %f", v, w);

	geometry_msgs::TwistStamped twistStampedMessage;
	twistStampedMessage.header.stamp = ros::Time::now();
	twistStampedMessage.header.frame_id = "fid";
	twistStampedMessage.twist.linear.x = v;
	twistStampedMessage.twist.linear.y = 0;
	twistStampedMessage.twist.linear.z = 0;
	twistStampedMessage.twist.angular.x = 0;
	twistStampedMessage.twist.angular.y = 0;
	twistStampedMessage.twist.angular.z = w;
	twistStamped.publish(twistStampedMessage);
}


int main(int argc, char **argv){
  	
	ros::init(argc, argv, "pose_parser");

	ros::NodeHandle n;

	message_filters::Subscriber<custom_messages::MotorSpeed> sub_fl(n, "motor_speed_fl", 1);
	message_filters::Subscriber<custom_messages::MotorSpeed> sub_fr(n, "motor_speed_fr", 1);
	message_filters::Subscriber<custom_messages::MotorSpeed> sub_rl(n, "motor_speed_rl", 1);
	message_filters::Subscriber<custom_messages::MotorSpeed> sub_rr(n, "motor_speed_rr", 1);

	typedef message_filters::sync_policies
		::ApproximateTime<custom_messages::MotorSpeed, custom_messages::MotorSpeed, custom_messages::MotorSpeed, custom_messages::MotorSpeed> MySyncPolicy;
	
	message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_fl, sub_fr, sub_rl, sub_rr);

    ros::Publisher twistStamped = n.advertise<geometry_msgs::TwistStamped>("twistStamped", 1000);
	sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, twistStamped));


  	ros::spin();

  return 0;
}


