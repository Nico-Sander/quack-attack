cmake_minimum_required(VERSION 2.8.3)
project(follow_lane)

find_package(catkin REQUIRED COMPONENTS
  rospy
  dynamic_reconfigure
)

# Generate dynamic reconfigure configuration files
generate_dynamic_reconfigure_options(
  cfg/DetectLane.cfg
  cfg/ControlLane.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)