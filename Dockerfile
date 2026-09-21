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

# System utilities, build tools, and USB/serial communication utilities.
# libsfml-dev builds the 2-D SFML simulator (arm_sim_sfml), the default view.
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
    x11-utils \
    libusb-1.0-0-dev \
    libudev-dev \
    udev \
    libsfml-dev \
    && rm -rf /var/lib/apt/lists/*

# Add the official ROS 2 GPG key and APT repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# ROS 2 Humble Base + build tooling. rclcpp-action is used by the arm
# controller/manual nodes (SetPosition action); ament-cmake-gtest builds the
# test suite in arm/test. Gazebo/rosbridge are NOT baked in anymore: they are
# optional and installed on demand (see arm/scripts/install_gazebo_harmonic.sh
# and arm/scripts/install_rosbridge.sh), keeping the image small.
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base \
    ros-humble-rclcpp-action \
    ros-humble-ament-cmake-gtest \
    ros-humble-xacro \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep for workspace dependency management
RUN rosdep init && rosdep update

# Run as root (no dedicated user); automatically source ROS 2 in interactive shells.
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

WORKDIR /root

# Container launcher (see run_arm.sh):
#   * real hardware by default (arm_system = controller + motor driver)
#   * "simulation" as the first argument runs the controller only
#   * any other argument(s) are executed as a shell command (dev shell/build)
COPY run_arm.sh /root/run_arm.sh
RUN chmod +x /root/run_arm.sh

ENTRYPOINT ["/root/run_arm.sh"]
CMD []