# Start from the official ROS 2 Jazzy Desktop image
FROM osrf/ros:jazzy-desktop

# Set environment variables
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# -----------------------------------------------------
# 1. Install System Tools & Python Dependencies
# -----------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    git \
    nano \
    python3-pip \
    python3-opencv \
    mesa-utils \
    libgl1 \
    libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# 2. Install ROS 2 Jazzy, Gazebo Harmonic, and Navigation
# -----------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-jazzy-nonpersistent-voxel-layer \
    ros-jazzy-ros-gz \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-localization \
    ros-jazzy-interactive-marker-twist-server \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-rmw-cyclonedds-cpp \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-velodyne \
    ros-jazzy-velodyne-description \
    ros-jazzy-ros2controlcli \
    ros-jazzy-laser-geometry \
    ros-jazzy-realsense2-description \
    ros-jazzy-slam-toolbox \
    ros-jazzy-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------
# 3. ROS Dependency & Workspace Setup
# -----------------------------------------------------
# Run rosdep update as root (or the default user)
RUN rosdep update --rosdistro jazzy

# Set the working directory (replaces 'cd ~/ros2_ws')
WORKDIR /home/ros2_ws

# COPY your local src directory into the container's workspace
# (Make sure your Dockerfile is next to your 'src' folder on your host machine)
COPY ./src ./src

# Install dependencies using rosdep
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------
# 4. Entrypoint Configuration
# -----------------------------------------------------
# Create an entrypoint script to handle environment sourcing automatically
RUN echo '#!/bin/bash\n\
source /opt/ros/jazzy/setup.bash\n\
if [ -f /home/ros2_ws/install/setup.bash ]; then\n\
    source /home/ros2_ws/install/setup.bash\n\
fi\n\
exec "$@"' > /entrypoint.sh && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]