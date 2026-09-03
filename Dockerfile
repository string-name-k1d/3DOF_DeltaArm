# Base image: Ubuntu 22.04 LTS
FROM ubuntu:22.04

# Set non-interactive mode during build to prevent prompt hangs
ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# Set standard locale required by ROS 2
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8

# Install system utilities, build tools, and USB/serial communication utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    build-essential \
    cmake \
    git \
    python3-pip \
    # Serial/USB utilities & hardware access libraries
    usbutils \
    minicom \
    libusb-1.0-0-dev \
    libudev-dev \
    udev \
    && rm -rf /var/lib/apt/lists/*

# Add the official ROS 2 GPG key and APT repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Base + rosdep + colcon
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rclcpp-action \
    ros-humble-ament-cmake-gtest \
    ros-humble-xacro \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Install Gazebo Classic (11) + the gazebo_ros_pkgs ROS 2 bridge (and related
# model/robot-state packages) so the arm simulation can run in the container.
# gazebo_ros_pkgs pulls in gazebo_ros, gazebo_ros2_control, gazebo_plugins, etc.
# libglm-dev is the math library the Gazebo sim (src/sim arm_sim_gazebo) uses.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-controller-manager \
    libglm-dev \
    && rm -rf /var/lib/apt/lists/*

# Install the ROS 2 to WebSocket bridge (rosbridge_suite): lets web browsers /
# external clients talk to the ROS 2 graph from the container (default port 9090).
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep for workspace dependency management
RUN rosdep init && rosdep update

# Create a non-root user and add them to dialout/plugdev groups for USB access
ARG USERNAME=rosuser
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    # Add user to dialout and plugdev groups to access /dev/ttyUSB*, /dev/ttyACM*, etc.
    && usermod -aG dialout,plugdev $USERNAME \
    # Grant sudo permissions for dev convenience
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && rm -rf /var/lib/apt/lists/*

# Automatically source ROS 2 environment for the user
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/${USERNAME}/.bashrc

USER $USERNAME
WORKDIR /home/${USERNAME}

# Default entrypoint starts a bash shell
CMD ["/bin/bash"]