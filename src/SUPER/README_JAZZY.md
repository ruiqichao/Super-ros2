# SUPER for ROS2 Jazzy

This is the SUPER (Safe Urban Perception and Exploration Robot) project adapted for ROS2 Jazzy Jalisco.

## Prerequisites

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- C++17 compatible compiler
- CMake >= 3.5
- PCL (Point Cloud Library)
- Eigen3
- yaml-cpp

## Installation

### 1. Install ROS2 Jazzy

Follow the official installation guide:
```bash
# Setup locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Jazzy
sudo apt update
sudo apt install ros-jazzy-desktop
sudo apt install python3-argcomplete
```

### 2. Install Dependencies

#### Automatic Installation (Recommended)

We provide an automatic installation script that will install all required dependencies:

```bash
cd /home/ruiqichao/ros2_ws/super_ws/src/SUPER
chmod +x install_dependencies.sh
./install_dependencies.sh
```

This script will:
- Install all ROS2 Jazzy dependencies
- Install system-level libraries (Eigen, PCL, OpenCV, etc.)
- Install Python dependencies
- Prepare the system for building the project

#### Manual Installation

Alternatively, you can install dependencies manually:

```bash
sudo apt update
sudo apt install \
  build-essential \
  cmake \
  git \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  libeigen3-dev \
  libpcl-dev \
  libyaml-cpp-dev \
  libdw-dev
```

### 3. Setup Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone the repository
git clone https://gitee.com/xiyue133/super_ros.git src/SUPER

# Install dependencies via rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install --packages-select super_planner rog_map

# Source the workspace
source install/setup.bash
```

## Usage

### Launch the Planner

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch the FSM node
ros2 launch super_planner rviz.launch.py
```

### Running the Planner

```bash
# Run the planner node
ros2 run super_planner fsm_node
```

## Configuration

The planner can be configured using YAML files located in `super_planner/config/`. You can modify parameters such as:
- Planning horizon
- Maximum velocity and acceleration
- Map resolution
- Trajectory optimization parameters

## Features

- Real-time trajectory planning
- Obstacle avoidance
- Multi-layered map representation (ROG-Map)
- Dynamic reconfiguration support
- RViz integration

## Troubleshooting

### Common Issues

1. **Package not found**: Make sure you have sourced the workspace (`source install/setup.bash`)

2. **Build errors**: Ensure all dependencies are installed and the correct ROS2 distribution is sourced

3. **Permission errors**: Make sure you have proper permissions for the workspace directory

### Building Specific Packages

If you only want to build specific packages:

```bash
# Build only super_planner
colcon build --packages-select super_planner

# Build with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Notes for ROS2 Jazzy

- This project has been tested with ROS2 Jazzy Jalisco
- Uses the FastDDS middleware by default
- Compatible with Python 3.12
- Uses ament_cmake build system

## Contributing

Please read the contribution guidelines in the main README.md file.

## License

This project is licensed under the BSD license - see the LICENSE file for details.
