#!/bin/bash
source /opt/ros/noetic/setup.bash
source /workspace/devel/setup.bash

roscore &
sleep 2

rosrun follow_lane dashboard_node.py &
rosrun follow_lane detect_lane_node.py &
rosrun follow_lane detect_intersection_node.py &
# rosrun obstacle_detection detect_obstacle_node.py &
# rosrun intersection_handling control_intersection_node &
# rosrun follow_lane switch_control_node.py &
sleep 2

rosrun follow_lane control_lane_node.py &

wait