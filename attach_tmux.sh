#!/bin/bash

CONTAINER_NAME="duckie_ros"
SESSION_NAME="duckie_session"

# Check if container is running
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo "❌ ERROR: Container '$CONTAINER_NAME' is not running."
    exit 1
fi

echo "🚀 Booting up tmux workspace..."

if [ -n "$TMUX" ]; then
    # Already inside a tmux session: create a new window
    echo "ℹ️  Already inside a tmux session. Creating a new window..."
    PANE_1=$(tmux new-window -d -P -F "#{pane_id}" -n "$CONTAINER_NAME" "docker exec -it $CONTAINER_NAME bash")
    SETUP_PANES=true
else
    # Not inside a tmux session: create or attach to a session
    if ! tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
        echo "ℹ️  Creating new tmux session '$SESSION_NAME'..."
        PANE_1=$(tmux new-session -d -P -F "#{pane_id}" -s "$SESSION_NAME" -n "$CONTAINER_NAME" "docker exec -it $CONTAINER_NAME bash")
        SETUP_PANES=true
    else
        echo "ℹ️  Attaching to existing tmux session '$SESSION_NAME'..."
        SETUP_PANES=false
    fi
fi

# Split the window into 4 panes if we just created it/the window
if [ "$SETUP_PANES" = true ]; then
    # Split horizontally (creates a right pane) and run docker command
    PANE_2=$(tmux split-window -h -P -F "#{pane_id}" -t "$PANE_1" "docker exec -it $CONTAINER_NAME bash")

    # Split the right pane vertically (creates bottom right)
    tmux split-window -v -t "$PANE_2" "docker exec -it $CONTAINER_NAME bash"

    # Select the very first pane (top left)
    tmux select-pane -t "$PANE_1"

    # Split the left pane vertically (creates bottom left)
    tmux split-window -v -t "$PANE_1" "docker exec -it $CONTAINER_NAME bash"

    # Move the focus back up to the top-left pane
    tmux select-pane -t "$PANE_1"
fi

# Activate/attach to the workspace
if [ -n "$TMUX" ]; then
    # Switch to the newly created window
    tmux select-window -t "$PANE_1"
    tmux select-pane -t "$PANE_1"
else
    # Attach your current terminal to the fully built session
    tmux attach-session -t "$SESSION_NAME"
fi

