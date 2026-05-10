cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  src/detect_obstacle_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)