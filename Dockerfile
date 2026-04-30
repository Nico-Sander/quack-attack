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

# 6. Auto-source environments for interactive terminal sessions
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /workspace/devel/setup.bash ]; then source /workspace/devel/setup.bash; fi" >> ~/.bashrc

# 7. Set the entrypoints
ENTRYPOINT [ "/entrypoint.sh" ]
