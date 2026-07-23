# DuckieRace: Quack Attack / Group 2

This repository contains the ROS Noetic workspace for the DuckieRace challenge. We use Docker to containerize the environment, ensuring consistent dependencies and a quick setup across all machines.

## Prerequisites
- [Docker](https://docs.docker.com/get-docker/) and Docker Compose plugin
- `avahi-daemon` (mDNS, so `<vehicle>.local` resolves on the host)
- `nmap` (fallback used by `start.sh` when mDNS fails)
- `tmux` (optional, but recommended for the multi-pane workflow)

<details>
<summary>Docker install guide for Ubuntu (tested on 24.04)</summary>

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

Verify the installation:

```shell
docker --version
docker compose version
```

**Step 4: Remove the need for running docker commands with sudo**

```shell
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```

</details>

---

## Initial Setup

Only needed once per machine (or after the `Dockerfile` changes).

```shell
git clone https://github.com/Nico-Sander/quack-attack.git
cd quack-attack
cp .env.example .env      # values are overwritten by start.sh at every run
docker compose build
```

## Daily Workflow

### 1. Grant display permissions (required for RViz, rqt, dashboards)

```shell
xhost +local:root
```

Put this in your `.bashrc` if you don't want to repeat it after every reboot.

### 2. Start the container

```shell
./start.sh <vehicle_name>     # e.g. ./start.sh trick   (default: track)
```

`start.sh` does the whole network setup for you:
- verifies you are on the `DuckieNetz` Wi-Fi (offers to connect if not)
- resolves the Duckiebot's IP via mDNS (`avahi-resolve`), falling back to an `nmap` subnet scan
- removes stale `/etc/hosts` pins that would shadow mDNS
- determines the host IP and exports `DUCKIEBOT_IP`, `HOST_IP`, `VEHICLE_NAME`, `HOST_UID`, `HOST_GID`
- starts the `duckie_ros` container in the background (`network_mode: host`, running as your user so files stay writable)

The container's `entrypoint.sh` runs `catkin_make` on startup and sources `devel/setup.bash`, so a fresh clone is ready to use. A failing build does **not** kill the container — you still get a shell to debug in.

### 3. Attach a terminal

**Option A — tmux, 4 panes (recommended):**

```shell
./attach_tmux.sh
```

**Option B — single shell (repeat in as many tabs as you need):**

```shell
docker exec -it duckie_ros bash
```

### 4. Build and run

Inside the container (`/workspace` is this repo, bind-mounted):

```shell
catkin_make              # only needed after changes; entrypoint already built once
source devel/setup.bash
```

Run a whole challenge stack:

```shell
roslaunch ch1_lane_following ch1_lane_following.launch
```

Or a single node:

```shell
rosrun ch1_lane_following detect_lane_node.py
```

### 5. Stop the container

```shell
docker compose down
```

## Useful Docker Commands

- `docker compose up -d` — start in the background **without** the network setup (Duckiebot will not be reachable; prefer `./start.sh`)
- `docker exec -it duckie_ros bash` — open an interactive shell in the running container
- `docker compose down` — stop and remove the container
- `docker compose restart` — restart (re-runs the entrypoint build)
- `docker compose logs -f` — stream container logs
- `docker compose build --no-cache` — full image rebuild, only after `Dockerfile` changes

---

## Packages

Catkin workspace under `src/packages/`. Every challenge is a self-contained package with its own nodes, config, model and launch file — challenges 2–4 build on the previous ones, but do not import from them.

| Package | Challenge | Launch |
| --- | --- | --- |
| `ch1_lane_following` | Lane following, stop at red lines | `roslaunch ch1_lane_following ch1_lane_following.launch` |
| `ch2_intersection_handling` | Read the intersection sign, stop, turn | `roslaunch ch2_intersection_handling ch2_intersection_handling.launch` |
| `ch3_obstacle_avoidance` | Reactive duckie avoidance (follow-the-gap) | `roslaunch ch3_obstacle_avoidance ch3_obstacle_avoidance.launch` |
| `ch4_mapping_pathfinding` | City mapping and timed gate run | `roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch` |
| `duckietown_msgs` | Message/service definitions used to talk to the nodes on the bot | — |

All nodes of ch1, ch2 and ch4 are namespaced under `VEHICLE_NAME` (override with `veh:=<name>`). ch3 runs un-namespaced.

### Common launch arguments

Every launch file documents its arguments inline; list them with `roslaunch --ros-args <pkg> <file>.launch`. The ones you use most:

```shell
# perception + state machine only, robot does not move
roslaunch ch1_lane_following ch1_lane_following.launch driving:=false

# camera/segmentation debug dashboard (needs X11)
roslaunch ch2_intersection_handling ch2_intersection_handling.launch dashboard:=true

# force every turn, to tune the manoeuvre without hunting for the right intersection
roslaunch ch2_intersection_handling ch2_intersection_handling.launch force_turn:=LEFT

# ch3: dry run — controller still plans and feeds the dashboard, but publishes no command
roslaunch ch3_obstacle_avoidance ch3_obstacle_avoidance.launch controller:=false
```

### Challenge 4 specifics

```shell
# mapping phase: explore every street, recording gates
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch start_edge:=A,4,D,2

# timed gate run, order announced on site as tag IDs
roslaunch ch4_mapping_pathfinding ch4_mapping_pathfinding.launch \
    mission_phase:=GATE_RUN start_edge:=C,4,F,2 gate_order:=7,5,11

# full mission without the robot: mapping node + fake driver, needs only a roscore
roslaunch ch4_mapping_pathfinding simulate.launch
```

A menu-driven front end walks through both phases in one long-lived launch (the map would be lost by restarting):

```shell
rosrun ch4_mapping_pathfinding mission_tui.py
```

See `src/packages/ch4_mapping_pathfinding/docs/workflow.md` for the manual equivalent.

### Machine Learning Model Training

The lane-segmentation networks (`models/*.pth`) are trained in a separate repository:

**[duckie-lane-segmentation](https://github.com/Nico-Sander/duckie-lane-segmentation)**

---

## Notes on Using the Duckiebot

- The web interface is at `http://<vehicle>` — not `http://<vehicle>.local`.

**Power on:** press the button on the battery and wait for it to boot. Check with `dts fleet discover` on the host.

**Power off:**
1. Preferred: use the web interface.
2. Otherwise press and hold the top button for ~20 seconds, then release.

**!IMPORTANT!** Never remove cables while the Duckiebot is on — this can corrupt its OS.
