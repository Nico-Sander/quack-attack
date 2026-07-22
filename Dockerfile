FROM ros:noetic-ros-base

ARG DEBIAN_FRONTEND=noninteractive

# 1. Install basic build tools and GUI dependencies
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    git \
    nano \
    vim \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-rviz \
    ros-noetic-compressed-image-transport \
    && rm -rf /var/lib/apt/lists/*

# 1. Install PyTorch and Deep Learning dependencies
# (We removed opencv-python-headless from this list)
RUN pip3 install --no-cache-dir --ignore-installed \
    "numpy<2.0" \
    torch torchvision \
    albumentations \
    segmentation-models-pytorch \
    ultralytics \
    pytest

# 1b. Explicit runtime dependencies of the ch4_mapping_pathfinding package.
# These used to arrive only transitively (networkx and matplotlib ride in on
# torch/ultralytics) or had to be pip-installed by hand inside a running
# container (pupil-apriltags). Declaring them here makes the image reproducible.
# Kept in a separate layer so editing this list does not invalidate the
# expensive torch layer above.
RUN pip3 install --no-cache-dir \
    networkx \
    matplotlib \
    pupil-apriltags \
    PyYAML

# 2. The GUI Fix!
# Albumentations automatically sneaks 'opencv-python-headless' in as a hidden dependency.
# We must explicitly uninstall it right after so ROS falls back to its native GUI-enabled cv2.
# This must stay AFTER every pip install above, since any of them may pull it back in.
RUN pip3 uninstall -y opencv-python-headless

WORKDIR /workspace

# 2. Initialize rosdep
RUN rosdep init || true

# 3. Copy source code
COPY src/ /workspace/src/

# 4. Install ros dependencies
RUN apt-get update && rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 5. Add the entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 6. Create a real user matching the host account.
# The container runs as the host UID/GID so files it writes into the bind-mounted
# repo come out owned by you. Without a matching /etc/passwd entry that UID is
# nameless, which is what produced "I have no name!" and the "groups: cannot find
# name for group ID" warning - and, more importantly, left the user with no home
# directory to read a .bashrc from.
# HOST_UID/HOST_GID are passed as build args by docker-compose (see build.args).
ARG HOST_UID=1000
ARG HOST_GID=1000
RUN if ! getent group "$HOST_GID" >/dev/null; then groupadd -g "$HOST_GID" duckie; fi && \
    if ! getent passwd "$HOST_UID" >/dev/null; then \
        useradd -u "$HOST_UID" -g "$HOST_GID" -m -s /bin/bash duckie; \
    fi

# Resolve the home directory of whichever account owns HOST_UID, so the .bashrc
# below lands in the right place even if the base image already had that UID.
# 7. Auto-source environments for interactive terminal sessions.
# docker exec starts a new shell and does NOT run the entrypoint, so this file is
# the only thing that sets up ROS for the tmux panes.
RUN USER_HOME=$(getent passwd "$HOST_UID" | cut -d: -f6) && \
    printf '%s\n' \
      'source /opt/ros/noetic/setup.bash' \
      'if [ -f /workspace/devel/setup.bash ]; then source /workspace/devel/setup.bash; fi' \
      >> "$USER_HOME/.bashrc" && \
    chown "$HOST_UID:$HOST_GID" "$USER_HOME/.bashrc"

# 7. Set the entrypoints
ENTRYPOINT [ "/entrypoint.sh" ]
