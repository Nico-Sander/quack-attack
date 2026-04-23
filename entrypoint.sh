#!/bin/bash
set -e

echo "=========================================="
echo " Starting ROS Workspace Initialization... "
echo "=========================================="

# Update package lists and rosdep
apt-get update
rosdep update

# Check and install missing dependencies from the mounted src/ folder
echo "Checking for missing dependencies in /workspace/src..."
rosdep install --from-paths src --ignore-src -r -y

echo "=========================================="
echo " Dependencies up to date!                 "
echo "=========================================="

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# If the workspace has already been built, source its overlay
if [ -f "/workspace/devel/setup.bash" ]; then
    source /workspace/devel/setup.bash
fi

# Execute the container's main command (e.g., 'sleep infinity')
exec "$@"