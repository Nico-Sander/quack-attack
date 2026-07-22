cmake_minimum_required(VERSION 3.0.2)
project(ch2_intersection_handling)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# No catkin_install_python on purpose. The nodes import their siblings
# (custom_enums, crossing) and read ../config/config.json and 52DB.yaml
# relative to __file__, so they only work when roslaunch starts them out of
# the source tree -- which is what it does, since it finds executables by
# scanning the package directory.
catkin_package()
