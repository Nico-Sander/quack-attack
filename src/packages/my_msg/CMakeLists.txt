cmake_minimum_required(VERSION 2.8.3)
project(my_msg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  OdomPosition.msg
)

catkin_package(
   CATKIN_DEPENDS
   message_runtime
)

catkin_package()