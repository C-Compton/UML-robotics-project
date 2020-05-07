cmake_minimum_required(VERSION 2.8.3)
project(sign_reader)

find_package(catkin REQUIRED COMPONENTS
  duckietown_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  SignInfo.msg
)

generate_messages(
  DEPENDENCIES
  duckietown_msgs geometry_msgs std_msgs
)

catkin_package(
  CATKIN_DEPENDS duckietown_msgs geometry_msgs roscpp rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
