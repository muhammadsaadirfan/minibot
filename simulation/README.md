# Mobile Robot Simulation for ROS Melodic

A complete ROS Melodic workspace featuring a custom 4-wheel differential drive mobile robot with autonomous navigation capabilities. This project provides a comprehensive simulation environment for testing and developing robotic navigation algorithms.

## 🤖 Robot Description

**TortoiseBot** - A custom 4-wheel differential drive mobile robot:
- **Dimensions**: 0.47m × 0.47m × 0.1m base
- **Wheel Configuration**: 4 caster wheels (front-left, front-right, rear-left, rear-right)
- **Wheel Specs**: 0.07m diameter, 0.05m width
- **Drive System**: Differential drive with custom hardware interface

## 📦 Packages

### mobile_robot
Core package containing robot description, hardware interface, and simulation launch files.

**Features:**
- Custom URDF robot model
- Differential drive hardware interface
- Gazebo simulation world
- SLAM mapping capabilities
- RViz visualization

### navigation
Navigation package with move_base configuration for autonomous navigation.

**Features:**
- Global and local costmaps
- Path planning and trajectory generation
- Obstacle avoidance
- Navigation stack integration

## 🚀 Installation

### Prerequisites
- Ubuntu 18.04 LTS
- ROS Melodic (Desktop-Full installation)
- Gazebo 9.x
- RViz

### Installation Steps

1. **Clone the repository:**
   ```bash
   git clone https://github.com/yourusername/mobile-robot-simulation.git
   cd mobile-robot-simulation
   ```

2. **Install dependencies:**
   ```bash
   sudo apt update
   sudo apt install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
   sudo apt install ros-melodic-controller-manager ros-melodic-diff-drive-controller
   sudo apt install ros-melodic-joint-state-controller ros-melodic-position-controllers
   sudo apt install ros-melodic-slam-gmapping ros-melodic-navigation
   sudo apt install ros-melodic-map-server ros-melodic-amcl
   ```

3. **Build the workspace:**
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## 🎮 Usage

### Basic Simulation
Launch the robot in Gazebo with SLAM mapping:
```bash
roslaunch mobile_robot sim_with_map.launch
```

### Navigation
Launch the navigation stack:
```bash
roslaunch navigation move_base.launch
```

### Individual Components

**Robot Description:**
```bash
roslaunch mobile_robot robot_description.launch
```

**Controllers:**
```bash
roslaunch mobile_robot controller.launch
```

**AMCL Localization:**
```bash
roslaunch mobile_robot amcl.launch
```

### RViz Visualization
RViz is automatically launched with the simulation. You can also launch it separately:
```bash
rviz -d src/mobile_robot/rviz/rviz.rviz
```

## 🗺️ Maps

The workspace includes several pre-built maps:
- `arena.pgm/yaml` - Arena environment
- `hcr.pgm/yaml` - HCR environment  
- `map.pgm/yaml` - Default map

## 🔧 Configuration

### Robot Parameters
- **Wheel Separation**: 0.30m
- **Wheel Radius**: 0.05m
- **Robot Footprint**: 0.5m × 0.5m
- **Inflation Radius**: 0.1m

### Controller Settings
Controller parameters can be modified in `src/mobile_robot/config/controllers.yaml`

### Navigation Parameters
Navigation stack parameters are in `src/navigation/config/`

## 🧪 Testing

### Manual Control
Use teleop to manually control the robot:
```bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Navigation Testing
1. Launch the simulation
2. Launch navigation stack
3. Use RViz to set navigation goals
4. Monitor robot behavior

## 📁 Project Structure

```
simulation/
├── src/
│   ├── mobile_robot/
│   │   ├── config/          # Controller configurations
│   │   ├── launch/          # Launch files
│   │   ├── map/            # Pre-built maps
│   │   ├── rviz/           # RViz configurations
│   │   ├── src/            # Source code
│   │   ├── urdf/           # Robot descriptions
│   │   └── worlds/         # Gazebo worlds
│   └── navigation/
│       ├── config/         # Navigation parameters
│       ├── launch/         # Navigation launch files
│       └── rviz/          # Navigation RViz config
├── devel/                  # Build artifacts
└── log/                   # ROS logs
```

## 🤝 Contributing

We welcome contributions! Please follow these steps:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

### Development Guidelines
- Follow ROS coding standards
- Add comments to complex code sections
- Update documentation for new features
- Test changes in simulation before submitting

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- ROS Community for the excellent robotics framework
- Gazebo team for the simulation environment
- Navigation stack contributors
- All contributors to this project

## 📞 Support

If you encounter any issues or have questions:
- Open an issue on GitHub
- Check the ROS Wiki for general ROS questions
- Review the troubleshooting section below

## 🔧 Troubleshooting

### Common Issues

**Gazebo not launching:**
```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find mobile_robot)/worlds
```

**Controller not loading:**
- Check if all dependencies are installed
- Verify controller configuration files
- Check ROS parameter server

**Navigation not working:**
- Ensure robot is localized (AMCL)
- Check costmap parameters
- Verify sensor data is being published

## 📊 Performance

- **Simulation Rate**: 1000Hz physics update
- **Controller Rate**: 50Hz
- **Navigation Update Rate**: 5Hz
- **SLAM Update Interval**: 5.0 seconds

---

**Happy Robotics Development! 🤖✨** 