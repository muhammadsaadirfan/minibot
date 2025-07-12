# Tutorial Guide

This tutorial will guide you through using the Mobile Robot Simulation project, from basic operation to advanced navigation techniques.

## 🚀 Getting Started

### Prerequisites
- Complete installation as described in [INSTALLATION.md](INSTALLATION.md)
- Basic understanding of ROS concepts
- Familiarity with terminal commands

### Quick Start
```bash
# Launch the complete simulation
roslaunch mobile_robot sim_with_map.launch
```

## 📚 Basic Tutorials

### Tutorial 1: Understanding the Robot

#### 1.1 Robot Description
The TortoiseBot is a 4-wheel differential drive robot with the following specifications:
- **Base Dimensions**: 0.47m × 0.47m × 0.1m
- **Wheel Configuration**: 4 caster wheels
- **Drive System**: Differential drive (left and right wheels)
- **Sensors**: Simulated laser scanner for navigation

#### 1.2 Exploring the URDF
```bash
# View the robot model in RViz
roslaunch mobile_robot robot_description.launch

# Check robot parameters
rosparam get /robot_description
```

#### 1.3 Understanding Topics
```bash
# List all active topics
rostopic list

# Monitor robot state
rostopic echo /joint_states

# Monitor laser scan data
rostopic echo /scan
```

### Tutorial 2: Manual Control

#### 2.1 Basic Teleoperation
```bash
# Terminal 1: Launch simulation
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Start teleop
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

**Teleop Controls:**
- `i` - Move forward
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `k` - Stop
- `Shift + i` - Move forward faster
- `Shift + ,` - Move backward faster

#### 2.2 Understanding Robot Movement
```bash
# Monitor velocity commands
rostopic echo /cmd_vel

# Monitor odometry
rostopic echo /odom

# Visualize robot movement in RViz
```

### Tutorial 3: Understanding the Simulation Environment

#### 3.1 Gazebo World
The simulation includes a custom world with:
- Walls and obstacles
- Different room layouts
- Realistic physics simulation

#### 3.2 Exploring the World
```bash
# Launch simulation
roslaunch mobile_robot sim_with_map.launch

# In Gazebo, you can:
# - Right-click and drag to rotate view
# - Scroll to zoom in/out
# - Shift + left-click to pan
```

#### 3.3 World Parameters
```bash
# Check world file location
rospack find mobile_robot
# Look in worlds/cylinder.world
```

## 🧭 Navigation Tutorials

### Tutorial 4: Basic Navigation

#### 4.1 Launching Navigation Stack
```bash
# Terminal 1: Launch simulation
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Launch navigation
roslaunch navigation move_base.launch
```

#### 4.2 Understanding Navigation Components
The navigation stack includes:
- **Global Costmap**: For global path planning
- **Local Costmap**: For local obstacle avoidance
- **Global Planner**: Plans paths to goals
- **Local Planner**: Executes smooth trajectories

#### 4.3 Setting Navigation Goals
1. Open RViz
2. Click on "2D Nav Goal" button
3. Click and drag on the map to set goal
4. Watch the robot navigate to the goal

### Tutorial 5: SLAM Mapping

#### 5.1 Creating a Map
```bash
# Terminal 1: Launch simulation with SLAM
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Manual control for mapping
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

#### 5.2 Mapping Process
1. **Start Mapping**: The SLAM node automatically starts
2. **Explore Environment**: Drive around to map the area
3. **Coverage**: Ensure all areas are scanned
4. **Save Map**: Use map_server to save the map

#### 5.3 Saving the Map
```bash
# Save the map (in a new terminal)
rosrun map_server map_saver -f my_map

# This creates my_map.pgm and my_map.yaml
```

### Tutorial 6: Localization with AMCL

#### 6.1 Using Pre-built Maps
```bash
# Terminal 1: Launch simulation with map
roslaunch mobile_robot sim_with_map.launch

# Terminal 2: Launch AMCL
roslaunch mobile_robot amcl.launch
```

#### 6.2 Robot Localization
1. **Initial Pose**: Set initial robot pose in RViz
2. **Particle Filter**: AMCL uses particle filter for localization
3. **Convergence**: Particles converge to robot's actual position

#### 6.3 Setting Initial Pose
1. Click "2D Pose Estimate" in RViz
2. Click on map where robot is located
3. Drag to set robot orientation
4. AMCL will localize the robot

## 🔧 Advanced Tutorials

### Tutorial 7: Customizing Robot Parameters

#### 7.1 Modifying Robot Dimensions
Edit `src/mobile_robot/urdf/mobile_robot.urdf`:
```xml
<!-- Change base dimensions -->
<link name="base_link">
  <visual>
    <geometry>
      <box size="0.5 0.5 0.12"/>  <!-- Modified dimensions -->
    </geometry>
  </visual>
</link>
```

#### 7.2 Adjusting Controller Parameters
Edit `src/mobile_robot/config/controllers.yaml`:
```yaml
diff_drive_controller:
  wheel_separation: 0.35  # Modified wheel separation
  wheel_radius: 0.06      # Modified wheel radius
```

#### 7.3 Updating Navigation Parameters
Edit `src/navigation/config/costmap_common_params.yaml`:
```yaml
footprint: [[0.3,0.3], [0.3,-0.3],[-0.3,-0.3],[-0.3,0.3]]  # Modified footprint
```

### Tutorial 8: Performance Optimization

#### 8.1 Optimizing Simulation Performance
```bash
# Reduce physics update rate
export GAZEBO_PHYSICS_UPDATE_RATE=500

# Limit Gazebo memory usage
export GAZEBO_MAX_MEMORY=2048

# Launch with reduced GUI
roslaunch mobile_robot sim_with_map.launch gui:=false
```

#### 8.2 Optimizing Navigation Performance
Edit navigation parameters for better performance:
```yaml
# In costmap_common_params.yaml
obstacle_range: 2.0        # Reduce sensor range
raytrace_range: 2.5        # Reduce ray tracing
inflation_radius: 0.08     # Reduce inflation
```

### Tutorial 9: Debugging and Troubleshooting

#### 9.1 Common Issues and Solutions

**Robot Not Moving:**
```bash
# Check controller status
rostopic echo /mobile_robot/controller_manager/status

# Check velocity commands
rostopic echo /cmd_vel

# Restart controllers
rosservice call /mobile_robot/controller_manager/switch_controller "start_controllers: ['diff_drive_controller']"
```

**Navigation Not Working:**
```bash
# Check costmap data
rostopic echo /move_base/global_costmap/costmap

# Check robot pose
rostopic echo /amcl_pose

# Verify sensor data
rostopic echo /scan
```

**SLAM Issues:**
```bash
# Check SLAM parameters
rosparam get /slam_gmapping

# Monitor SLAM status
rostopic echo /slam_gmapping/status
```

#### 9.2 Debugging Tools
```bash
# RViz for visualization
rviz -d src/mobile_robot/rviz/rviz.rviz

# Plot data
rqt_plot /cmd_vel/linear/x /cmd_vel/angular/z

# Monitor topics
rostopic hz /scan
```

## 🎯 Practical Exercises

### Exercise 1: Basic Navigation
1. Launch the simulation
2. Set a navigation goal
3. Observe the robot's path planning
4. Try different goal locations

### Exercise 2: Obstacle Avoidance
1. Add obstacles to the environment
2. Set navigation goals
3. Observe how the robot avoids obstacles
4. Test different obstacle configurations

### Exercise 3: Mapping and Localization
1. Create a new map using SLAM
2. Save the map
3. Use the map for localization
4. Test navigation with the new map

### Exercise 4: Parameter Tuning
1. Modify robot parameters
2. Test navigation performance
3. Optimize for your specific use case
4. Document your findings

## 📊 Monitoring and Analysis

### Performance Metrics
```bash
# Monitor CPU usage
htop

# Monitor memory usage
free -h

# Monitor disk usage
df -h

# Monitor network (if using distributed setup)
iftop
```

### ROS Metrics
```bash
# Topic frequency
rostopic hz /scan

# Message size
rostopic bw /scan

# Latency
rostopic delay /scan
```

## 🔗 Integration with Other Tools

### RViz Plugins
- **Robot Model**: Visualize robot geometry
- **Laser Scan**: View sensor data
- **Path**: Visualize planned paths
- **Pose Array**: View AMCL particles
- **Map**: Display occupancy grid

### RQT Tools
```bash
# Plot data
rqt_plot

# Monitor topics
rqt_graph

# Parameter management
rqt_reconfigure
```

## 📝 Best Practices

### 1. Development Workflow
- Always test changes in simulation first
- Use version control for configuration changes
- Document parameter modifications
- Test with different scenarios

### 2. Performance Optimization
- Monitor system resources
- Optimize parameters for your hardware
- Use appropriate update rates
- Consider headless operation for production

### 3. Debugging
- Use RViz for visualization
- Monitor relevant topics
- Check log files for errors
- Use ROS debugging tools

## 🎓 Next Steps

After completing these tutorials:

1. **Experiment with Parameters**: Try different robot configurations
2. **Add Sensors**: Integrate additional sensors
3. **Custom Behaviors**: Implement custom navigation behaviors
4. **Multi-Robot**: Extend to multi-robot scenarios
5. **Real Hardware**: Adapt for real robot hardware

## 📞 Getting Help

- Check the [troubleshooting section](#troubleshooting)
- Search existing GitHub issues
- Create new issues with detailed information
- Join the community discussions

---

**Happy Learning! 🤖📚** 