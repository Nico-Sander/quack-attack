# DuckieRace: Quack Attack

This repository contains the ROS Noetic workspace for the DuckieRace challenge. We use Docker to containerize the environment, ensuring consistent dependencies and a quick setup across all machines.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

---

## Setup Instructions

### 1. Clone the repository
Clone the project directly to your host machine:
```bash
git clone [https://github.com/Nico-Sander/quack-attack.git](https://github.com/Nico-Sander/quack-attack.git)
cd quack-attack
```

### 2. Configure network
Copy the `.env.example` file to `.env` and adjust `HOST_IP` to your local IP Adress of you device.

### 3. Build the Docker Image and start the container
```bash
docker compose up -d --build
```

### 4. Enter the container
```bash
docker exec -it duckie_ros bash
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

## Common Commands
- Rebuild Image:
```bash
docker compose build --no-cache
```

## code structure
This reposistory is formed as a catkin workspace. The code is seprated in packages. The actual code for the DuckieRace challenge is in src/package/follow_lane/src. The package duckietown_msgs contains message definitions for the communication with the nodes running on the duckiebot. 

