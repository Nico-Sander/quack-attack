#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun odometry odometry_node.py &
rosrun odometry control_point_node.py &

wait