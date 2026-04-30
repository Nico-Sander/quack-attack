# Troubleshooting ROS 1 Hostname Resolution in Duckietown

## 1. The Symptom
When starting the Docker workspace, connecting to the ROS master succeeds, and `rostopic list` displays all hardware topics broadcasted from the Duckiebot (e.g., `/trick/camera_node/image/compressed`). However, when attempting to measure bandwidth with `rostopic hz /trick/camera_node/image/compressed` or visualizing the feed in RViz, no data is received and the connection hangs silently.

## 2. The Root Cause: ROS 1 Peer-to-Peer Networking
The problem stems from how ROS 1 handles data transport versus master registration. ROS 1 architecture utilizes a centralized Master for node registration but establishes peer-to-peer TCP connections for the actual data transfer.

* **Master Registration:** `rostopic list` contacts the Master using the explicitly provided IP address defined in `ROS_MASTER_URI` (e.g., `http://${DUCKIEBOT_IP}:11311`). Because this uses a direct IP, it succeeds.
* **Data Transport:** When you subscribe to a topic via `rostopic hz` or RViz, the Master does not forward the data. Instead, it provides the client with the URI of the Publisher node running on the Jetson Nano. 
* **The Failure Point:** The Publisher node usually registers with the Master using its hostname (e.g., `http://trick.local:43212`). If the client (your Host PC / Docker container) cannot resolve `trick.local` back to an IP address, the peer-to-peer TCP handshake fails.

## 3. The `.lan` vs `.local` Mismatch (Network Deep Dive)
In a typical Duckietown environment utilizing a GL.iNet router, a split-brain DNS situation occurs:

* **The Router's DNS (`.lan`):** The GL.iNet router acts as the DHCP server. When the Duckiebot connects, the router assigns an IP (e.g., `192.168.90.229`) and appends its default domain, mapping it as `trick.lan`.
* **The Robot's mDNS (`.local`):** The Duckiebot runs an Ubuntu-based OS, which utilizes Avahi/mDNS to broadcast its presence on the local subnet. It shouts its hostname as `trick.local`.

When the ROS node starts, it registers with the Master as `trick.local`. However, when your Host PC asks the router for the IP of `trick.local`, the router fails to resolve it because its internal tables only contain `trick.lan`.

## 4. Troubleshooting the Network Infrastructure
If you suspect DNS or IP resolution issues, you can trace the network topology using a specific sequence of commands to uncover the hostname mismatch.

### Step 4.1: Identify Your Host Network Interface
First, determine the IP address of your host machine and the active network interface.
```bash
ip -4 addr show
```
*   **What to look for:** Find the active wireless interface (e.g., `wlp2s0` or `wlan0`). Note your assigned IP address (e.g., `192.168.90.182`) and the subnet (indicated by `/24`).

### Step 4.2: Identify the Gateway (Router)
Check the ARP (Address Resolution Protocol) table to identify the router managing the local network.
```bash
arp -a
```
*   **What to look for:** Look for the gateway IP, usually ending in `.1` (e.g., `192.168.90.1`). Note the hostname associated with it (e.g., `console.gl-inet.com`). This confirms which device is acting as the DHCP server and assigning `.lan` suffixes.

### Step 4.3: Scan the Subnet for Devices
Run an `nmap` sweep across the entire subnet to discover all connected devices and the hostnames assigned to them by the router.
```bash
sudo nmap -sn 192.168.90.0/24
```
*   *(Replace `192.168.90.0/24` with the subnet identified in Step 4.1)*
*   **What to look for:** Locate the target vehicle (e.g., `trick`). You will likely see the router has identified it as `trick.lan` (e.g., `Nmap scan report for trick.lan (192.168.90.229)`). This confirms the split-brain DNS issue if the robot expects to be reached at `.local`.

## 5. The Docker Complication
Your Docker setup utilizes `network_mode: "host"`. This is a standard and necessary configuration for ROS 1 hardware to prevent port-forwarding headaches and allow the container to access ephemeral TCP/UDP ports. 

However, `network_mode: "host"` causes the container to bypass Docker's internal DNS resolver and share the Host PC's network stack. Because of this override, the `extra_hosts` directive defined in the `docker-compose.yml` (`${VEHICLE_NAME}.lan:${DUCKIEBOT_IP}` and `${VEHICLE_NAME}:${DUCKIEBOT_IP}`) is frequently ignored by the container. If the Host PC cannot resolve the name, the container cannot resolve it either.

## 6. The Secondary Issue: RViz Decompression Plugins
Even if the network resolution is flawless, visualizing the camera feed will fail if the container lacks the correct decompression libraries. 

The Duckiebot transmits camera frames via the `sensor_msgs/CompressedImage` message type to conserve Wi-Fi bandwidth. The baseline `ros-noetic-rviz` installation defined in the `Dockerfile` only supports raw uncompressed image streams. Without explicitly installing `ros-noetic-compressed-image-transport`, RViz will connect to the publisher but silently fail to decode the frames.

## 7. The Bulletproof Solution
To ensure stable connections regardless of the router's configuration, the startup sequence must handle the DNS resolution dynamically.

1. **Dynamic IP Discovery:** The `start.sh` script executes network scans (`getent`, `nmcli`, `nmap`) to accurately locate the Duckiebot's dynamic IP on the subnet.
2. **`/etc/hosts` Universal Translation:** Once the IP is found, the script injects a universal mapping directly into the Host PC's `/etc/hosts` file, associating the IP with all possible variants: `trick`, `trick.local`, and `trick.lan`. Because Docker runs in host network mode, it instantly inherits this foolproof mapping.
3. **Automated Docker Builds:** By updating the `start.sh` script to run `docker compose up -d --build`, any changes to dependencies in the `Dockerfile` (like adding the compressed image transport plugin) are automatically compiled using the verified, dynamically exported environment variables.
