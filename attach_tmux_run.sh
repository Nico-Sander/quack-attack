#!/bin/bash

CONTAINER_NAME="duckie_ros"
SESSION_NAME="duckie_session"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ ERROR: Container '$CONTAINER_NAME' is not running."
    exit 1
fi

echo "🚀 Booting up tmux workspace..."

# Define the common ROS setup command to avoid typing it repeatedly
ROS_SETUP="source /opt/ros/noetic/setup.bash && source devel/setup.bash"

# Create a new tmux session in the background
tmux new-session -d -s $SESSION_NAME "docker exec -it $CONTAINER_NAME bash"

# Split horizontally (creates a right pane)
tmux split-window -h "docker exec -it $CONTAINER_NAME bash"

# Split the right pane vertically (creates bottom right)
tmux split-window -v "docker exec -it $CONTAINER_NAME bash"

# Select the left pane (using -L is safer than hardcoded index numbers)
tmux select-pane -L

# Split the left pane vertically (creates bottom left)
tmux split-window -v "docker exec -it $CONTAINER_NAME bash"

# --- SEND COMMANDS TO PANES ---

# 1. Top Left Pane
tmux select-pane -U
tmux send-keys "$ROS_SETUP && rosrun follow_lane detect_lane_node.py" C-m

# 2. Bottom Left Pane
tmux select-pane -D
tmux send-keys "$ROS_SETUP && rosrun follow_lane detect_intersection_node.py" C-m

# 3. Bottom Right Pane
tmux select-pane -R
tmux send-keys "$ROS_SETUP && sleep 5 && rosrun follow_lane control_lane_node.py"

# 4. Top Right Pane
tmux select-pane -U
tmux send-keys "$ROS_SETUP && rosrun follow_lane switch_control_node.py" C-m

# Move the focus back to the top-left pane before attaching
tmux select-pane -D

# Attach your current terminal to the fully built session
tmux attach-session -t $SESSION_NAME