#!/bin/bash

# Configuration
TARGET_SSID="DuckieNetz"
# Override per-run without editing this file: ./start.sh dorette
VEHICLE_NAME="${1:-track}"
# .local is the mDNS name the bot announces itself under, so it always tracks
# the current DHCP lease. .lan comes from the router's DHCP-DNS, which keeps
# serving records for leases that have already been handed to another bot.
VEHICLE_DOMAIN=".local"

echo "=========================================="
echo " 🦆 Pre-flight Check: Duckiebot Network   "
echo "=========================================="

# 1. Check current Wi-Fi network
CURRENT_SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d':' -f2)

if [ "$CURRENT_SSID" != "$TARGET_SSID" ]; then
    echo "❌ You are not connected to the '$TARGET_SSID' network."
    echo "   Currently connected to: ${CURRENT_SSID:-None}"

    # NEW: Check if the network is actually in range
    echo "📡 Scanning for nearby networks..."

    # nmcli lists visible SSIDs. grep -q silently checks for an exact match.
    if nmcli -t -f ssid dev wifi | grep -q "^${TARGET_SSID}$"; then
        echo "✅ '$TARGET_SSID' is in range."

        read -p "❓ Do you want to try auto-connecting to $TARGET_SSID now? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            read -s -p "🔑 Enter Wi-Fi Password for $TARGET_SSID: " WIFI_PASS
            echo
            echo "⏳ Attempting to connect..."
            nmcli dev wifi connect "$TARGET_SSID" password "$WIFI_PASS"

            if [ $? -ne 0 ]; then
                echo "💥 ERROR: Failed to connect to $TARGET_SSID. Please check the password or connect manually."
                exit 1
            fi
            echo "✅ Successfully connected to $TARGET_SSID."
        else
            echo "🛑 Aborting. Please connect to the network manually and run this script again."
            exit 1
        fi
    else
        # If the network is not found in the scan
        echo "🛑 ERROR: '$TARGET_SSID' is not in range."
        echo "   Please move closer to the router, ensure it is powered on, and try again."
        exit 1
    fi
else
    echo "✅ Network check passed. Connected to $TARGET_SSID."
fi

# 2. Clear any stale pin for this vehicle BEFORE resolving.
# /etc/hosts is consulted ahead of mDNS (see `hosts:` in /etc/nsswitch.conf), so
# a leftover entry from an earlier run silently shadows the live mDNS answer and
# the lookup below would just read back a dead IP this script wrote itself.
if grep -qE "[[:space:]]$VEHICLE_NAME(\.|[[:space:]]|$)" /etc/hosts; then
    echo "🧹 Removing stale /etc/hosts pin for '$VEHICLE_NAME' (it shadows mDNS)..."
    echo "   (You may be prompted for your sudo password)"
    sudo sed -i.bak -E "/[[:space:]]$VEHICLE_NAME(\.|[[:space:]]|$)/d" /etc/hosts
fi

# 3. Automatically find the Duckiebot IP
echo "🔍 Locating Duckiebot ($VEHICLE_NAME$VEHICLE_DOMAIN)..."

# avahi-resolve queries mDNS directly and ignores /etc/hosts entirely; getent is
# the fallback and is safe now that the pin above is gone.
if command -v avahi-resolve >/dev/null 2>&1; then
    DUCKIEBOT_IP=$(avahi-resolve -4 -n "$VEHICLE_NAME$VEHICLE_DOMAIN" 2>/dev/null | awk '{print $2}')
else
    DUCKIEBOT_IP=$(getent ahosts $VEHICLE_NAME$VEHICLE_DOMAIN | awk '/STREAM/{print $1; exit}')
fi

if [ -z "$DUCKIEBOT_IP" ]; then
    echo "⚠️ Fast DNS resolution failed. Identifying local subnet for scanning..."

    WIFI_IFACE=$(nmcli -t -f DEVICE,TYPE connection show --active | grep 802-11-wireless | cut -d':' -f1 | head -n 1)

    if [ -z "$WIFI_IFACE" ]; then
        echo "💥 ERROR: Could not determine the active Wi-Fi interface to perform a scan."
        exit 1
    fi

    SUBNET=$(ip route show dev $WIFI_IFACE | awk '/proto kernel/ {print $1}')

    echo "📡 Engaging nmap scan on subnet $SUBNET..."
    # nmap reverse-resolves via the router, which answers with .lan (not .local),
    # so match the bare vehicle name. The IP is on the same line as the name.
    DUCKIEBOT_IP=$(nmap -sn $SUBNET \
        | grep -E "Nmap scan report for $VEHICLE_NAME(\.|[[:space:]]|\()" \
        | grep -oE "([0-9]{1,3}\.){3}[0-9]{1,3}" | head -n 1)
fi

# 4. Hard Abort if Duckiebot is unreachable
if [ -z "$DUCKIEBOT_IP" ]; then
    echo "💥 ERROR: Could not locate Duckiebot ($VEHICLE_NAME$VEHICLE_DOMAIN) on the network."
    echo "   Ensure the Duckiebot is powered on, booted up, and connected to '$TARGET_SSID'."
    exit 1
fi

echo "✅ Found Duckiebot at $DUCKIEBOT_IP"

# 4. Automatically find the Host PC IP
HOST_IP=$(ip route get $DUCKIEBOT_IP | awk -F"src " 'NR==1{split($2,a," ");print a[1]}')

if [ -z "$HOST_IP" ]; then
    echo "💥 ERROR: Could not determine host IP. Are you sure you have a valid network connection?"
    exit 1
fi
echo "✅ Host IP identified as $HOST_IP"

# 5. Verify Hostname Resolution
echo "=========================================="
echo " 🔧 Verifying Hostname Resolution...      "
echo "=========================================="
# ROS 1 nodes on the Jetson Nano register with their hostname (<vehicle>.local),
# so both the host (network_mode: "host") and the container must resolve it.
#
# We deliberately do NOT write this mapping into /etc/hosts. A static pin is read
# ahead of mDNS, so it outlives the DHCP lease it was based on and then shadows
# the correct answer - which is what silently broke ssh and ROS_MASTER_URI on any
# bot whose lease had moved. avahi already resolves .local dynamically; we only
# confirm it agrees with the IP we just discovered.
RESOLVED_IP=$(getent ahosts "$VEHICLE_NAME.local" | awk '/STREAM/{print $1; exit}')

if [ "$RESOLVED_IP" = "$DUCKIEBOT_IP" ]; then
    echo "✅ $VEHICLE_NAME.local resolves to $DUCKIEBOT_IP via mDNS."
elif [ -z "$RESOLVED_IP" ]; then
    echo "⚠️  Host cannot resolve $VEHICLE_NAME.local (is avahi-daemon running?)."
    echo "   ROS peer-to-peer topic subscriptions will fail on the host."
    echo "   Check with: systemctl status avahi-daemon"
else
    echo "⚠️  $VEHICLE_NAME.local resolves to $RESOLVED_IP but the bot is at $DUCKIEBOT_IP."
    echo "   Something is still shadowing mDNS - check /etc/hosts for a leftover entry."
fi

# 6. Export variables to the shell environment
export DUCKIEBOT_IP=$DUCKIEBOT_IP
export HOST_IP=$HOST_IP
export VEHICLE_NAME=$VEHICLE_NAME

# So the container runs as you, not root - otherwise every file it writes
# into the bind-mounted repo (calibration yaml, build/, devel/) comes out
# root-owned and you need `sudo chown` before you can touch it again.
export HOST_UID=$(id -u)
export HOST_GID=$(id -g)

# 7. Launch Docker Compose
echo "=========================================="
echo " 🔎 Verifying Docker Configuration...     "
echo "=========================================="
echo "The shell environment currently holds:"
echo " -> DUCKIEBOT_IP: $DUCKIEBOT_IP"
echo " -> HOST_IP: $HOST_IP"
echo " -> VEHICLE_NAME: $VEHICLE_NAME"
echo " -> HOST_UID:HOST_GID: $HOST_UID:$HOST_GID"
echo ""
echo "Here is what Docker Compose will actually use for the ROS Master and IPs:"

# Run config and filter for the exact lines we care about to prove it worked
docker compose config | grep -E 'ROS_MASTER_URI|ROS_IP|extra_hosts' -A 2

echo ""
echo "=========================================="
echo " 🚀 Starting ROS Workspace Container...   "
echo "=========================================="
# (Optional: Add a pause here if you want to manually approve it before it runs)
# read -p "Press Enter to launch or Ctrl+C to abort..."

docker compose up -d --force-recreate

echo "🟢 Container is running! Your environment is fully configured."
