# DuckieRace

## Setup Virtual Box with ros noetic

## Setuo workspace

## Launch Ros nodes

```
source /opt/ros/noetic/setup.bash    

export ROS_MASTER_URI=http://tick.local:11311
export ROS_IP=192.168.137.78
```

Build the project
```
catkin_make 
source devel/setup.bash
rosrun follow_lane detect_lane_node.py
```


