cmake_minimum_required(VERSION 2.8.3)
project(jetsoncar_msgs)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs actionlib_msgs)

# Inspired from https://github.com/ros/common_msgs/tree/jade-devel/nav_msgs

add_message_files(
  DIRECTORY msg
  FILES
  Test.msg
  Encoders.msg
  Setpoint.msg
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs actionlib_msgs)

catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs actionlib_msgs)
