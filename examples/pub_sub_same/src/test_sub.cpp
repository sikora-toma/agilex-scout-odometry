#include "ros/ros.h"
#include "std_msgs/String.h"


class pub_sub {

public:
  std_msgs::String message1;
  std_msgs::String message2;

private:
  ros::NodeHandle n; 

  ros::Subscriber sub;
  ros::Subscriber sub2;
  ros::Publisher pub; 
  ros::Timer timer1;
  
public:
  pub_sub(){
    sub = n.subscribe("/chatter", 1, &pub_sub::callback_m1, this);
    sub2 = n.subscribe("/chatter2", 1, &pub_sub::callback_m2, this);
    pub = n.advertise<std_msgs::String>("/rechatter", 1);
    timer1 = n.createTimer(ros::Duration(1), &pub_sub::callback_t, this);
  }

  void callback_m1(const std_msgs::String::ConstPtr& msg){
    message1=*msg;
  }

  void callback_m2(const std_msgs::String::ConstPtr& msg){
    message2=*msg;
  }

  void callback_t(const ros::TimerEvent&) {
    pub.publish(message1);
    pub.publish(message2);
    ROS_INFO("Callback 1 triggered");
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "subscribe_and_publish");
  
  pub_sub my_pub_sub;
  
  ros::spin();
  
  return 0;
}
