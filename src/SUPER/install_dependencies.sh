#!/bin/bash

echo "Installing ROS2 Jazzy dependencies for the SUPER project..."

echo "Updating package lists..."
sudo apt update

# 安装ROS2 Jazzy基础依赖
echo "Installing ROS2 Jazzy base dependencies..."
sudo apt install -y \
    ros-jazzy-desktop \
    ros-jazzy-rclcpp \
    ros-jazzy-std-msgs \
    ros-jazzy-sensor-msgs \
    ros-jazzy-geometry-msgs \
    ros-jazzy-nav-msgs \
    ros-jazzy-tf2-ros \
    ros-jazzy-visualization-msgs \
    ros-jazzy-pcl-conversions \
    ros-jazzy-mavros-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip

# 安装其他系统依赖
echo "Installing system dependencies..."
sudo apt install -y \
    build-essential \
    cmake \
    git \
    libeigen3-dev \
    libsuitesparse-dev \
    libqt5-widgets5 \
    libqt5-opengl5-dev \
    libqt5-concurrent5 \
    libqglviewer-dev-qt5 \
    freeglut3-dev \
    libglew-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libpcl-dev \
    pcl-tools \
    libproj-dev \
    libyaml-cpp-dev \
    libzmq3-dev \
    libspdlog-dev \
    libopencv-dev \
    libnlopt-dev \
    libceres-dev

# 安装Python依赖
echo "Installing Python dependencies..."
pip3 install numpy scipy pyquaternion transforms3d matplotlib

# 初始化rosdep
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo "Dependencies installation completed!"
echo ""
echo "Now you can build the project with:"
echo "cd /home/ruiqichao/ros2_ws"
echo "source /opt/ros/jazzy/setup.bash"
echo "colcon build --symlink-install"
echo ""
echo "For detailed build instructions, please refer to README_JAZZY.md"
