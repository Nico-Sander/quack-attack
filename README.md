# DuckieRace

## Setup Virtual Box with ros noetic

Download Ubuntu 20.04 image from https://www.releases.ubuntu.com/focal/
Setup a virtual machine with the image. Set the network adapter to bridged adapter so that the VM uses the same Network as the host.
Follow the instructions from https://wiki.ros.org/noetic/Installation/Ubuntu to setup ros noetic.
Clone this github repository and change the remote head to your own github repository.
```
git clone https://github.com/DuckieBotIRAS/DuckieRace_2026.git
git remote set-url <your-github-repository-url>
```

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
For multiple nodes you can write launchers and run them like
```
launchers/follow_lane.sh
```

## code structure
This reposistory is formed as a catkin workspace. The code is seprated in packages. The actual code for the DuckieRace challenge is in src/package/follow_lane/src. The package duckietown_msgs contains message definitions for the communication with the nodes running on the duckiebot. 

