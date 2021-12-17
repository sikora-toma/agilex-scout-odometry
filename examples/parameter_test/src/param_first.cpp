#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv){

  ros::init(argc, argv, "param_first");
  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("parameter", 1000);
  std::string name;
  n.getParam("/name", name);

  ros::Rate loop_rate(10);

  while (ros::ok()){

    std_msgs::String msg;


    msg.data = name;

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }

  return 0;
}
