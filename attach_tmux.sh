#!/bin/bash

CONTAINER_NAME="duckie_ros"
SESSION_NAME="duckie_session"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ ERROR: Container '$CONTAINER_NAME' is not running."
    exit 1
fi

echo "🚀 Booting up tmux workspace..."

# Create a new tmux session in the background and run the docker command in the first pane
tmux new-session -d -s $SESSION_NAME "docker exec -it $CONTAINER_NAME bash"

# Split horizontally (creates a right pane) and run docker command
tmux split-window -h "docker exec -it $CONTAINER_NAME bash"

# Split the right pane vertically (creates bottom right)
tmux split-window -v "docker exec -it $CONTAINER_NAME bash"

# Select the very first pane (top left)
tmux select-pane -t 1

# Split the left pane vertically (creates bottom left)
tmux split-window -v "docker exec -it $CONTAINER_NAME bash"

# Attach your current terminal to the fully built session
tmux attach-session -t $SESSION_NAME
