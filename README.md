# DuckieRace: Quack Attack / Group 2

This repository contains the ROS Noetic workspace for the DuckieRace challenge. We use Docker to containerize the environment, ensuring consistent dependencies and a quick setup across all machines.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)
- `tmux` (Optional, but highly recommended for multi-pane terminal workflow)

### Docker Install Guide for Ubuntu (tested on 24.04)

**Step 1: Clean up any old installations or leftovers**:

```shell
for pkg in docker.io docker-doc docker-compose docker-compose-v2 podman-docker containerd runc; do sudo apt-get remove $pkg; done
```

**Step 2: Set up Docker's official repository**:

1. Update `apt` and install required tools:
    ```shell
    sudo apt-get update
    sudo apt-get install ca-certificates curl
    ```

2. Add Docker's official GPG key:
    ```shell
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    ```

3. Add the repository to your Apt sources:
    ```shell
    echo \
        "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
        $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update
    ```

**Step 3: Install Docker Engine and Docker Compose**
    
```shell
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Verifiy installation:

```shell
docker --version
docker compose version
```

**Step 4: Remove the need for running docker commands with sudo**

1. Create the docker group

    ```shell
    sudo groupadd docker
    ```

2. Add your user to the group

    ```shell
    sudo usermod -aG docker $USER
    ```

3. Apply new group membership

    ```shell
    newgrp docker
    ```

---

## Initial Setup Instructions

You only need to do this once when setting up the project on a new machine or when the `Dockerfile` has changed.

### 1. Clone the repository

Clone the project directly to your host machine:

```shell
git clone [https://github.com/Nico-Sander/quack-attack.git](https://github.com/Nico-Sander/quack-attack.git)
cd quack-attack
```

### 2. Build the Docker Image
Compile the custom ROS Noetic image (which includes all tools needed for running the rosnodes used in this project). This is rarely necessary unless system-level dependencies in the `Dockerfile` are changed

```bash
docker compose build
```


## Daily Workflow

Think of the container as a lightweigth, pre-configured virtual machine. You "turn it on" in the background, and then you "attach" terminals to it to do your work.

### 1. Grant Display Permissions (Required for GUI tools)

Before starting the container, allow it to draw windows (like RViz or rqt) on your host machine's screen.

```shell
xhost +local:root
```

If you don't want to run this step after each restart of your system, consider putting this command into your `.bashrc` file.

### 2. Start the Container

Set the name of the Duckiebot you want to control in `./start.sh`, then run the automated startup script:

```shell
./start.sh
```

This script handles the heavy-lifting automatically:
- Verifies connection to the DuckieNetz Wi-Fi network (and attempts auto-connection if needed)
- Scans the network to dynamically locate the Duckiebot's and the Host's IP adresses.
- Injects the correct hostname mappings into your /etc/hosts file to allow seamless ROS peer-to-peer communication.
- Boots the duckie_ros container in the background

### 3. Attach to the Workspace

Once the container is running, plug a terminal session into it. Choose the method that fits your workflow:

**Option A: The Tmux Multi-Pane Setup** (Recommended)

```shell
./attach_tmux.sh
```

This script automatically generates a background tmux session, splits your terminal into 4 distinct panes, and attaches all of them to the running duckie_ros container simultaneously. Perfect for running multiple ROS nodes at once.

**Option B: The Standard Single Terminal**

```shell
docker exec -it duckie_ros bash
```
Use this to open a single, standard bash terminal inside the running container. You can run this command in as many new terminal tabs as you need.

### 4. Build and Run ROS Nodes

Once inside the container, you are in a standard Ubuntu ROS Noetic environment. The entrypoint.sh script automatically sources the /opt/ros/noetic/setup.bash and your local workspace overlays. Your code from your host machine is automatically synced here via volumes.

Build the project and source the workspace:

```shell
catkin_make
source devel/setup.bash
```

Run a single node:

```shell
rosrun <package_name> <node_name>.py
rosrun follow_lane detect_lane.py
```

Run a launch script for multiple nodes

```shell
launchers/<launch_script_name>.sh
launchers/follow_lane.sh
```

### 5. Stop the Container

```shell
docker compose down
```

## Common / Useful Docker Commands

- `docker compose up -d`: Starts the container in the background. (Networking configuration will not be correct, and Duckiebot will not be reachable)
- `docker exec -it duckie_ros bash`: Opens an interactive bash terminal inside the running container.
- `docker compose down`: Stops and removes the running container safely.
- `docker compose restart`: Quickly restarts the container. Useful if you changed variables in your `.env` file and need them to apply.
- `docker compose logs -f`: Streams the backgroung logs of the container. Press `CTRL+C` to exit the log view.
- `docker compose build --no-cache`: Forces a complete rebuild of the Docker image from scratch. Use this only if you update the `Dockerfile` with new system-level dependencies.

## Code Structure

This repository is formed as a Catkin workspace. The code is separated into packages.

- `src/packages/follow_lane/src`: Contains the actual code for the DuckieRace challenge.
- `src/packages/duckietown_msgs`: Contains custom message definitions required for communicating with the nodes running directly on the Duckiebot.

### Machine Learning Model Training
The neural network models used by the `follow_lane` nodes in this repository are trained separately. If you need to adjust the dataset, retrain the model, or view the training pipeline, please visit our standalone ML training repository:

**[duckie-lane-segmentation](https://github.com/Nico-Sander/duckie-lane-segmentation)**

## Important Notes on using the duckiebot
- Webinterface is accessible via: http://trick not http://trick.local

### Power on duckiebot
- Press the button on the battery and wait for the duckiebot to boot up.
- Check the status of the duckiebot by running `dts fleet discover` on the host machine

### Powering off the duckiebot
1. Preferred: Use the webinterface 
2. Press and hold the top button for ~20 seconds, then release.

**!IMPORTANT!** Do not remove any cables while the duckiebot is on, since this can cause curruption of the duckiebot's OS
