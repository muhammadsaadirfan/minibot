# Installation Guide

This guide provides detailed instructions for installing and setting up the Mobile Robot Simulation project on your system.

## System Requirements

### Minimum Requirements
- **Operating System**: Ubuntu 18.04 LTS (Bionic Beaver)
- **RAM**: 4GB minimum, 8GB recommended
- **Storage**: 2GB free disk space
- **Graphics**: OpenGL 3.0 compatible graphics card
- **Processor**: Intel i3 or equivalent

### Recommended Requirements
- **Operating System**: Ubuntu 18.04 LTS
- **RAM**: 8GB or more
- **Storage**: 5GB free disk space
- **Graphics**: Dedicated graphics card with 2GB+ VRAM
- **Processor**: Intel i5 or equivalent

## Prerequisites

### 1. Install Ubuntu 18.04 LTS
If you haven't already installed Ubuntu 18.04 LTS, download it from the [official Ubuntu website](https://ubuntu.com/download/desktop).

### 2. Install ROS Melodic
Follow the official ROS Melodic installation guide:

```bash
# Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Install ROS Melodic Desktop-Full
sudo apt update
sudo apt install ros-melodic-desktop-full

# Initialize rosdep
sudo rosdep init
rosdep update

# Setup environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install dependencies for building packages
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 3. Install Additional ROS Packages
Install the required ROS packages for this project:

```bash
# Update package list
sudo apt update

# Install Gazebo packages
sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control

# Install controller packages
sudo apt install ros-melodic-controller-manager ros-melodic-diff-drive-controller
sudo apt install ros-melodic-joint-state-controller ros-melodic-position-controllers

# Install navigation packages
sudo apt install ros-melodic-navigation ros-melodic-slam-gmapping
sudo apt install ros-melodic-map-server ros-melodic-amcl

# Install additional utilities
sudo apt install ros-melodic-teleop-twist-keyboard
sudo apt install ros-melodic-rviz ros-melodic-rqt-plot
```

### 4. Install Development Tools
Install additional development tools that might be needed:

```bash
# Install build tools
sudo apt install build-essential cmake git

# Install Python development tools
sudo apt install python-pip python-dev

# Install additional Python packages
pip install --user numpy matplotlib
```

## Installation Steps

### 1. Clone the Repository
```bash
# Navigate to your desired directory
cd ~/catkin_ws/src

# Clone the repository
git clone https://github.com/yourusername/mobile-robot-simulation.git

# Navigate to the workspace
cd ~/catkin_ws
```

### 2. Install Dependencies
```bash
# Install ROS dependencies
rosdep install --from-paths src --ignore-src -r -y

# Install any missing system dependencies
sudo apt install -f
```

### 3. Build the Workspace
```bash
# Build the workspace
catkin_make

# Source the workspace
source devel/setup.bash
```

### 4. Verify Installation
Test that everything is working correctly:

```bash
# Test robot description
roslaunch mobile_robot robot_description.launch

# Test simulation (in a new terminal)
roslaunch mobile_robot sim_with_map.launch
```

## Post-Installation Setup

### 1. Environment Setup
Add the workspace to your bashrc for automatic sourcing:

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. Gazebo Model Path
Set the Gazebo model path to include custom models:

```bash
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$(rospack find mobile_robot)/worlds" >> ~/.bashrc
source ~/.bashrc
```

### 3. RViz Configuration
Copy RViz configurations to your home directory (optional):

```bash
cp src/mobile_robot/rviz/rviz.rviz ~/.rviz/
cp src/navigation/rviz/nav.rviz ~/.rviz/
```

## Troubleshooting

### Common Issues

#### 1. Build Errors
If you encounter build errors:

```bash
# Clean build directory
catkin_make clean

# Rebuild
catkin_make
```

#### 2. Missing Dependencies
If packages are missing:

```bash
# Update package list
sudo apt update

# Install missing packages
sudo apt install ros-melodic-[package-name]

# Reinstall dependencies
rosdep install --from-paths src --ignore-src -r -y
```

#### 3. Gazebo Not Launching
If Gazebo fails to launch:

```bash
# Check Gazebo installation
gazebo --verbose

# Reset Gazebo
rm -rf ~/.gazebo/

# Reinstall Gazebo
sudo apt install --reinstall gazebo9 libgazebo9-dev
```

#### 4. RViz Issues
If RViz has problems:

```bash
# Reset RViz configuration
rm -rf ~/.rviz/

# Reinstall RViz
sudo apt install --reinstall ros-melodic-rviz
```

#### 5. Controller Issues
If controllers fail to load:

```bash
# Check controller manager
rosrun controller_manager controller_manager list

# Check controller status
rostopic echo /mobile_robot/controller_manager/status
```

### Performance Optimization

#### 1. Graphics Performance
For better graphics performance:

```bash
# Install proprietary drivers (if available)
sudo ubuntu-drivers autoinstall

# Optimize Gazebo settings
export GAZEBO_IP=127.0.0.1
```

#### 2. Memory Optimization
For systems with limited RAM:

```bash
# Reduce Gazebo physics update rate
export GAZEBO_PHYSICS_UPDATE_RATE=500

# Limit Gazebo memory usage
export GAZEBO_MAX_MEMORY=2048
```

## Verification

### 1. Basic Functionality Test
Run the basic simulation test:

```bash
# Terminal 1: Launch simulation
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Check topics
rostopic list

# Terminal 3: Check robot state
rostopic echo /joint_states
```

### 2. Navigation Test
Test the navigation functionality:

```bash
# Terminal 1: Launch simulation
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Launch navigation
roslaunch navigation move_base.launch

# Terminal 3: Manual control
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### 3. SLAM Test
Test SLAM mapping:

```bash
# Terminal 1: Launch simulation with SLAM
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Manual control for mapping
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

## Next Steps

After successful installation:

1. **Read the README.md** for usage instructions
2. **Explore the launch files** to understand the system
3. **Try the tutorials** in the documentation
4. **Join the community** for support and contributions

## Support

If you encounter issues during installation:

1. Check the [troubleshooting section](#troubleshooting)
2. Search existing [GitHub issues](https://github.com/yourusername/mobile-robot-simulation/issues)
3. Create a new issue with detailed information
4. Contact the maintainers for assistance

---

**Happy Robotics Development! 🤖✨** 