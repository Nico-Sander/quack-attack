# DuckieRace

## Setup Virtual Box with ros noetic

```
sudo apt install ros-noetic-image-transport-plugins
```

## Setuo workspace

## Launch Ros nodes

```
source /opt/ros/noetic/setup.bash    

export ROS_MASTER_URI=http://tick.local:11311
export ROS_IP=192.168.137.66
export VEHICLE_NAME=tick
```

Build the project
```
catkin_make 
source devel/setup.bash
```

Run the nodes
For a single node
```
rosrun follow_lane detect_lane_node.py
```
for multiple nodes
```
launchers/follow_lane.sh
```


