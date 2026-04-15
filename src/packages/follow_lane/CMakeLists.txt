cmake_minimum_required(VERSION 2.8.3)
project(follow_lane)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)