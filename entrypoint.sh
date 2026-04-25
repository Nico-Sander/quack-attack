#!/bin/bash
set -e

echo "=========================================="
echo " Starting ROS Workspace Initialization... "
echo "=========================================="

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# If the workspace has already been built, source its overlay
if [ -f "/workspace/devel/setup.bash" ]; then
    source /workspace/devel/setup.bash
fi

# Execute the container's main command (e.g., 'sleep infinity')
exec "$@"