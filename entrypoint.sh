#!/bin/bash
set -e

echo "=========================================="
echo " Starting ROS Workspace Initialization... "
echo "=========================================="

# Source the ROS environment
source /opt/ros/noetic/setup.bash

# Build the workspace on startup. Incremental builds are quick when nothing
# changed, so this is cheap on a normal restart and means a fresh clone works
# without a manual catkin_make first.
#
# Deliberately not fatal: `set -e` would tear the container down on a compile
# error, taking away the very shell you would use to read the error. Instead we
# warn and carry on, so `attach_tmux.sh` still gets you a working prompt.
echo "🔨 Building catkin workspace..."
if (cd /workspace && catkin_make); then
    echo "✅ Workspace built."
else
    echo "⚠️  catkin_make failed - container is still up so you can debug."
    echo "   Re-run 'catkin_make' inside /workspace after fixing the error."
fi

# If the workspace built (now or previously), source its overlay
if [ -f "/workspace/devel/setup.bash" ]; then
    source /workspace/devel/setup.bash
fi

# Execute the container's main command (e.g., 'sleep infinity')
exec "$@"