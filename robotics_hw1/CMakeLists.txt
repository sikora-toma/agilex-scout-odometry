cmake_minimum_required(VERSION 3.0.2)
project(robotics_hw1)

find_package(
  catkin REQUIRED
  COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    message_generation
    geometry_msgs
    message_filters
    dynamic_reconfigure
    tf2_ros
)

add_message_files(
  FILES
    MotorSpeed.msg
    OdometryCustom.msg
)

add_service_files(
  FILES
  ResetOdometry.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
)

generate_dynamic_reconfigure_options(
cfg/parameters.cfg
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    nav_msgs
    message_runtime
    geometry_msgs
    message_filters
)


## Build modules
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(odometry_calculation src/odometry_calculation.cpp)
add_dependencies(odometry_calculation robotics_hw1_generate_messages_cpp)
add_dependencies(odometry_calculation ${PROJECT_NAME}_gencfg)
target_link_libraries(odometry_calculation ${catkin_LIBRARIES})

add_executable(pose_parser src/pose_parser.cpp)
add_dependencies(pose_parser robotics_hw1_generate_messages_cpp)
target_link_libraries(pose_parser ${catkin_LIBRARIES})
