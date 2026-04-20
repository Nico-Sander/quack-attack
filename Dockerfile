FROM ros:noetic-ros-base

# 1. Install basic build tools and rosdep tools
RUN apt-get update && apt-get install -y \
    python3-catkin-tools \
    python3-pip \
    python3-rosdep \
    git \
    nano \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# 2. Initialize and update rosdep
RUN rosdep init || true
RUN rosdep update

# 3. Copy ONLY your source code into the build environment
COPY src/ ./src/

# 4. Use rosdep to install everything listed in your package.xml files
RUN apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 5. Source the ROS environment automatically
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
