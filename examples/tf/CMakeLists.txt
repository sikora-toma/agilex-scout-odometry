cmake_minimum_required(VERSION 2.8.3)
project(tf_examples)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf tf2 tf2_ros rospy turtlesim)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(tf_pub src/pub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})

add_executable(tf2_pub src/pub_tf2.cpp)
target_link_libraries(tf2_pub ${catkin_LIBRARIES})

