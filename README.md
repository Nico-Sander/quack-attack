# DuckieRace: Quack Attack

This repository contains the ROS Noetic workspace for the DuckieRace challenge. We use Docker to containerize the environment, ensuring consistent dependencies and a quick setup across all machines.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/)
- [Docker Compose](https://docs.docker.com/compose/install/)

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

You only need to do this once when setting up the project on a new machine.

### 1. Clone the repository

Clone the project directly to your host machine:

```shell
git clone [https://github.com/Nico-Sander/quack-attack.git](https://github.com/Nico-Sander/quack-attack.git)
cd quack-attack
```

### 2. Configure network

Copy the `.env.example` file to `.env` and adjust `HOST_IP` to your local IP Adress of you device. Also ensure `DUCKIEBOT_IP` and `VEHICLE_NAME` match your target Duckiebot.

### 3. Build the Docker Image and start the container
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

### 2. Start the Container

This turns on your ROS environment in the background (`-d` stands for detached mode). It will stay running until you explicitly stop it.

```shell
docker compose up -d
```

### 3. Open a Terminal inside the Container

This plugs a terminal sesion into the running container. **You can run dhis command in as many new terminal tabs as you need** (e.g., one for each ROS Node)

```shell
docker exec -it duckie_ros bash
```

### 4. Build and Run ROS Nodes

Once inside the container, you are in a standard Ubuntu ROS Noetic environment. Your code from you host machine is automatically synced here.

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

### 4. Enter the container
```bash
docker exec -it duckie_ros bash
```

## Common / Useful Docker Commands

- `docker compose up -d`: Starts the container in the background.
- `docker exec -it duckie_ros bash`: Opens an interactive bash terminal inside the running container.
- `docker compose down`: Stops and removes the running container safely.
- `docker compose restart`: Quickly restarts the container. Useful if you changed variables in your `.env` file and need them to apply.
- `docker compose logs -f`: Streams the backgroung logs of the container. Press `CTRL+C` to exit the log view.
- `docker compose build --no-cache`: Forces a complete rebuild of the Docker image from scratch. Use this only if you update the `Dockerfile` with new system-level dependencies.

## code structure

This repository is formed as a Catkin workspace. The code is separated into packages.

- `src/packages/follow_lane/src`: Contains the actual code for the DuckieRace challenge.
- `src/packages/duckietown_msgs`: Contains custom message definitions required for communicating with the nodes running directly on the Duckiebot.
- `src/packages/duckie_visualizer`: Contains custom nodes for viewing live camera feeds and telemetry data.
