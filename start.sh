#!/bin/bash

# Configuration
TARGET_SSID="Duckienetz"
VEHICLE_NAME="trick"
VEHICLE_DOMAIN=".lan"

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

# 2. Automatically find the Duckiebot IP
echo "🔍 Locating Duckiebot ($VEHICLE_NAME$VEHICLE_DOMAIN)..."

DUCKIEBOT_IP=$(getent ahosts $VEHICLE_NAME$VEHICLE_DOMAIN | awk '{ print $1 }' | head -n 1)

if [ -z "$DUCKIEBOT_IP" ]; then
    echo "⚠️ Fast DNS resolution failed. Identifying local subnet for scanning..."
    
    WIFI_IFACE=$(nmcli -t -f DEVICE,TYPE connection show --active | grep 802-11-wireless | cut -d':' -f1 | head -n 1)
    
    if [ -z "$WIFI_IFACE" ]; then
        echo "💥 ERROR: Could not determine the active Wi-Fi interface to perform a scan."
        exit 1
    fi

    SUBNET=$(ip route show dev $WIFI_IFACE | awk '/proto kernel/ {print $1}')
    
    echo "📡 Engaging nmap scan on subnet $SUBNET..."
    DUCKIEBOT_IP=$(nmap -sn $SUBNET | grep "$VEHICLE_NAME$VEHICLE_DOMAIN" -A 1 | grep -oE "\b([0-9]{1,3}\.){3}[0-9]{1,3}\b")
fi

# 3. Hard Abort if Duckiebot is unreachable
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

# 5. Export variables to the shell environment
export DUCKIEBOT_IP=$DUCKIEBOT_IP
export HOST_IP=$HOST_IP
export VEHICLE_NAME=$VEHICLE_NAME

# 6. Launch Docker Compose
echo "=========================================="
echo " 🚀 Starting ROS Workspace Container...   "
echo "=========================================="
docker compose up -d

echo "🟢 Container is running! Your environment is fully configured."