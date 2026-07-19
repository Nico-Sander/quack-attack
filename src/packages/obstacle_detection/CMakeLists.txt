cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detection)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  duckietown_msgs
  dynamic_reconfigure
)

# Reads config/control_lane_node.json at build time, so the sliders inherit the
# same defaults, ranges and help text the node runs with.
generate_dynamic_reconfigure_options(
  cfg/ControlLane.cfg
)

catkin_package(
  CATKIN_DEPENDS dynamic_reconfigure
)

catkin_install_python(PROGRAMS
  src/control_lane_node.py
  src/dashboard_node.py
  src/detect_lane_node.py
  src/detect_obstacle_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)