#!/bin/bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rosrun follow_lane detect_lane_node.py &
rosrun follow_lane switch_control_node.py &
sleep 5

rosrun follow_lane control_lane_node.py &
wait