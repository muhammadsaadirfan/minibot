# Minibot: From Chassis to Code — Learn Mobile Robotics Step-by-Step


![alt text](header.png)
---

## Introduction

#### _This comprehensive guide will show you how to build a complete educational mobile robot from scratch using ROS 1 Melodic, Jetson Nano, and Arduino. You'll craft hardware assemblies, write custom ROS nodes, integrate LIDAR navigation, and actually make your robot move autonomously like a true robotics scholar. Yes, yes—there'll be launch files, YAML configs, rosserial communication, and other ancient rituals. The robot still won't thank you. But your students will. Definitely._

---

## What is the _Minibot_ Educational Platform?

The **Minibot** isn't just another robot kit—it's a complete educational ecosystem designed to teach hardware integration, embedded programming, and AI/robotics concepts using **ROS 1 Melodic**. This platform bridges the gap between theory and practice by providing:

- Real-world hardware integration experience
- **ROS-based software architecture** understanding  
- Practical navigation and SLAM implementation
- Low-cost, accessible component selection

In a nutshell, the Minibot acts as your gateway to professional robotics development. Every component has been carefully selected to balance educational value, cost-effectiveness, and real-world applicability.

Imagine robotics education is a vast ocean. The **Minibot** is your well-equipped vessel that takes you from the shallow waters of theory to the deep seas of autonomous navigation. The platform comes with comprehensive documentation that guides you through every wave and current you'll encounter.

Yes, it might sound complex. But don't worry! This tutorial walks you through every single step—from selecting components to running autonomous navigation. We'll build everything modularly, so you can understand each subsystem before integrating the complete system. It's educational, it's practical, and honestly......it's the robot platform you wish existed when you started learning robotics!

![alt text](system.png)

Before we dive into the build process, here's an overview of the Minibot's key subsystems:

1. **Processing Unit (Jetson Nano)**:
   - Runs ROS 1 Melodic master node
   - Handles high-level navigation algorithms
   - Processes LIDAR data for SLAM
   - Manages system coordination and visualization

2. **Motor Control Subsystem**:
   - Arduino UNO for real-time motor control
   - L298N motor driver for power management
   - Encoder feedback for precise odometry
   - rosserial communication with ROS

3. **Sensor Integration**:
   - LIDAR for navigation and obstacle detection
   - Encoders for dead reckoning
   - Battery monitoring for system health
   - LCD display for status information

4. **Navigation Stack**:
   - SLAM using gmapping
   - Path planning with move_base
   - Costmap-based obstacle avoidance
   - tf tree for coordinate transformations

---

## Table of Contents

1. Introduction & Project Overview
2. Project Structure
3. Bill of Materials (BOM)
4. System Architecture
5. Hardware Assembly Guide
6. Circuit Diagrams & Wiring
7. Software Installation
8. Arduino Firmware Setup
9. ROS Integration
10. LIDAR Integration
11. Navigation Stack Setup
12. SLAM Configuration
13. Testing & Debugging
14. Complete System Launch
15. Future Improvements
16. Troubleshooting

### Quick Navigation
- Hardware Requirements
- Software Requirements
- Installation Guide
- Usage Instructions
- Configuration
- Troubleshooting

---

## Introduction & Project Overview

### Who This Guide Is For

This comprehensive tutorial is designed for:

- **Students** learning robotics and ROS fundamentals
- **Robotics enthusiasts** wanting hands-on experience
- **Developers** transitioning into robotics
- **Educators** seeking a complete teaching platform

### What You'll Learn

By the end of this guide, you'll have:

✅ **Built a complete mobile robot** from individual components  
✅ **Mastered ROS 1 Melodic** installation and configuration  
✅ **Implemented rosserial communication** between ROS and Arduino  
✅ **Created custom ROS nodes** for motor control and navigation  
✅ **Integrated LIDAR** for autonomous navigation  
✅ **Configured SLAM** for real-time mapping  
✅ **Tuned navigation parameters** for optimal performance  
✅ **Debugged common robotics issues** you'll encounter in real projects

### Educational Philosophy

_"The best way to learn robotics is to build robots."_

This guide follows a hands-on approach where every concept is immediately applied to the physical system. You won't just read about coordinate transforms—you'll implement them. You won't just study SLAM algorithms—you'll watch your robot build maps in real-time.

---

## Project Structure

This project implements a complete ROS-based mobile robot system with hardware control, navigation, and simulation capabilities. The robot uses ROS Control framework for motor control, integrates with Arduino for low-level hardware interface, and includes full navigation stack configuration.

```
ros_control_final/
├── minibot/
│   ├── firmware.ino                    # Arduino firmware for motor control
│   └── src/
│       ├── mobile_robot/              # Main robot package
│       │   ├── config/
│       │   │   └── controllers.yaml   # ROS Control configuration
│       │   ├── launch/
│       │   │   ├── controller.launch  # Hardware interface launch
│       │   │   ├── amcl.launch        # Localization
│       │   │   ├── robot_description.launch
│       │   │   ├── sim_with_map.launch # Simulation with map
│       │   │   └── world.launch       # Gazebo world
│       │   ├── map/                   # Pre-built maps
│       │   ├── src/
│       │   │   └── diff_drive_hardware_interface.cpp # Hardware interface
│       │   ├── urdf/                  # Robot description files
│       │   │   ├── meshes/            # 3D models
│       │   │   ├── my_robo_hardware.urdf
│       │   │   └── my_robo_simulation.urdf
│       │   └── worlds/                # Gazebo world files
│       └── navigation/                # Navigation package
│           ├── config/                # Navigation parameters
│           ├── launch/
│           │   └── move_base.launch   # Navigation stack
│           └── rviz/                  # RViz configurations
```

### Key Features

#### Hardware Interface
- **ROS Control Integration**: Custom hardware interface for differential drive robot
- **Arduino Communication**: rosserial-based communication with Arduino for motor control
- **Encoder Feedback**: Real-time encoder reading for odometry
- **Motor Control**: PWM-based motor control with smoothing and deadband

#### Navigation System
- **Move Base**: Complete navigation stack with global and local planning
- **Costmap Configuration**: Optimized costmaps for obstacle avoidance
- **Path Planning**: DWA local planner with configurable parameters
- **Localization**: AMCL for robot localization

#### Simulation Support
- **Gazebo Integration**: Full simulation environment with custom world
- **URDF Models**: Detailed robot description with 3D meshes
- **Map Support**: Pre-built maps for testing and development

---

## Bill of Materials (BOM)

### Core Hardware Components

| Component | Quantity | Purpose | Estimated Cost |
|-----------|----------|---------|----------------|
| **Jetson Nano** | 1 | Main processing unit for ROS 1 Melodic | $99 |
| **Arduino UNO** | 1 | Motor control and sensor interface | $25 |
| **L298N Motor Driver** | 1 | Drives two DC motors with current protection | $8 |
| **16x2 LCD Display** | 1 | System status and debugging information | $10 |
| **Encoder Motors (2x)** | 2 | DC motors with built-in encoders for odometry | $40 |
| **RPLIDAR A1M8** | 1 | 360° laser scanner for navigation | $99 |
| **Battery Pack (7.4V LiPo)** | 1 | Power supply for entire system | $30 |
| **Robot Chassis Kit** | 1 | Mechanical platform and wheels | $25 |
| **Jumper Wires & Connectors** | 1 set | Electrical connections | $15 |

**Total Estimated Cost: ~$351**

### Tools Required

- Soldering iron and solder
- Multimeter for electrical testing
- Screwdriver set
- Wire strippers
- Hot glue gun (optional)
- 3D printer access (optional, for custom mounts)

### Software Requirements

| Software | Version | Purpose |
|----------|---------|---------|
| **Ubuntu** | 18.04 LTS | Operating system for ROS 1 Melodic |
| **ROS 1** | Melodic | Robotic middleware framework |
| **Arduino IDE** | 1.8+ | Programming Arduino firmware |
| **Python** | 2.7/3.6 | ROS node development |

---

### Communication Architecture

The Minibot uses a distributed communication architecture:

1. **ROS Master Node** (Jetson Nano)
   - Coordinates all system components
   - Runs navigation algorithms
   - Processes sensor data

2. **Arduino Subsystem** (Microcontroller)
   - Real-time motor control
   - Encoder data acquisition
   - rosserial communication bridge

3. **Sensor Integration**
   - LIDAR data streaming
   - Odometry computation
   - System health monitoring

### Data Flow Diagram

```
LIDAR ──► ROS Topics ──► SLAM Node ──► Map
  │                        │           │
  │                        ▼           ▼
  └──► Obstacle ──► Path Planner ──► Navigation
      Detection              │         Commands
                             │           │
                             ▼           ▼
                        Arduino ──► Motor Driver ──► Wheels
                             ▲           
                             │           
                        Encoders ──► Odometry ──► ROS Topics
```

---

## Hardware Assembly Guide

### Step 1: Chassis Preparation

First, let's prepare the mechanical foundation:

```bash
# Tools needed for this step:
# - Screwdriver set
# - Drill (if custom mounting holes needed)
```

1. **Unpack the chassis kit** and identify all components
2. **Mount the motors** to the chassis using provided brackets
3. **Install wheels** and ensure smooth rotation
4. **Create mounting points** for electronics (Jetson Nano, Arduino, battery)

### Step 2: Electronics Mounting

Strategic component placement is crucial for:
- Heat dissipation (Jetson Nano)
- Wire management
- Accessibility for debugging
- Weight distribution

**Recommended Layout:**
```
     [LIDAR]
        │
[Battery]──[Jetson Nano]
    │          │
[Arduino]──[L298N]──[Motors]
    │
  [LCD]
```

### Step 3: Power Distribution

Create a robust power distribution system:

1. **Main Power Bus**: 7.4V from LiPo battery
2. **5V Rail**: For Arduino, LCD, and logic circuits  
3. **12V Rail**: For motor driver (if using 12V motors)
4. **3.3V Rail**: For sensor interfaces

**Power Budget Calculation:**
- Jetson Nano: ~10W (2A @ 5V)
- Arduino UNO: ~0.5W (100mA @ 5V)
- Motors: ~24W (2A @ 12V each)
- LIDAR: ~2.5W (500mA @ 5V)
- **Total: ~37W peak consumption**

### Step 4: Component Integration

Install each major component with consideration for:

**Jetson Nano Placement:**
- Adequate ventilation for heat dissipation
- Access to GPIO pins for sensor connections
- USB ports for LIDAR and Arduino communication

**Arduino Positioning:**
- Close to motor driver for short signal paths
- Access to programming port
- Clear view of status LEDs

**LIDAR Mounting:**
- 360° unobstructed view
- Secure mounting to prevent vibration
- Appropriate height for navigation tasks

---

## Circuit Diagrams & Wiring

### Master Wiring Diagram

```
Jetson Nano GPIO Layout:
Pin 1  (3.3V)  ──► Logic Level Shifters
Pin 6  (GND)   ──► Common Ground
Pin 8  (GPIO14)──► Arduino Communication (Optional)
Pin 10 (GPIO15)──► Status LED

Arduino UNO Connections:
Digital Pin 2  ──► Motor 1 Encoder A
Digital Pin 3  ──► Motor 1 Encoder B  
Digital Pin 4  ──► Motor 2 Encoder A
Digital Pin 5  ──► Motor 2 Encoder B
Digital Pin 6  ──► Motor 1 PWM (L298N)
Digital Pin 7  ──► Motor 1 Direction 1
Digital Pin 8  ──► Motor 1 Direction 2
Digital Pin 9  ──► Motor 2 PWM (L298N)
Digital Pin 10 ──► Motor 2 Direction 1
Digital Pin 11 ──► Motor 2 Direction 2

Analog Pin A4  ──► LCD SDA
Analog Pin A5  ──► LCD SCL
```

### L298N Motor Driver Connections

```
L298N Pinout:
IN1, IN2 ──► Motor 1 Direction Control (from Arduino D7, D8)
IN3, IN4 ──► Motor 2 Direction Control (from Arduino D10, D11)
ENA      ──► Motor 1 PWM Speed Control (from Arduino D6)
ENB      ──► Motor 2 PWM Speed Control (from Arduino D9)
OUT1, OUT2 ──► Motor 1 Terminals
OUT3, OUT4 ──► Motor 2 Terminals
VCC      ──► 7.4V Battery Positive
GND      ──► Common Ground
5V OUT   ──► 5V Supply for Arduino (if needed)
```

### LIDAR Connection

```
RPLIDAR A1M8:
Red Wire    ──► 5V Power Supply
Black Wire  ──► Ground
Green Wire  ──► USB Data+ (via USB adapter)
White Wire  ──► USB Data- (via USB adapter)

Connection: LIDAR ──► USB Hub ──► Jetson Nano USB Port
```

### Power Distribution Schematic

```
Battery (7.4V LiPo)
    │
    ├──► L298N Motor Driver (7.4V Input)
    │       │
    │       └──► 5V Regulator Output ──► Arduino VIN
    │
    └──► Buck Converter (7.4V → 5V, 3A)
            │
            ├──► Jetson Nano (5V, 2A)
            ├──► LIDAR (5V, 500mA)
            └──► LCD Display (5V, 50mA)
```

---

## Software Installation

### Installing Ubuntu 18.04 on Jetson Nano

1. **Download Jetson Nano Developer Kit SD Card Image**
   ```bash
   # Download from NVIDIA Developer website
   wget https://developer.nvidia.com/jetson-nano-sd-card-image
   ```

2. **Flash SD Card** using Etcher or dd command
   ```bash
   # Using dd (replace /dev/sdX with your SD card)
   sudo dd if=jetson-nano-image.img of=/dev/sdX bs=4M status=progress
   ```

3. **Initial Setup**
   - Insert SD card and boot Jetson Nano
   - Complete Ubuntu setup wizard
   - Connect to WiFi/Ethernet
   - Update system packages

### Installing ROS 1 Melodic

```bash
# Step 1: Setup sources.list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Step 2: Setup keys
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Step 3: Update package index
sudo apt update

# Step 4: Install ROS Melodic Desktop Full
sudo apt install ros-melodic-desktop-full

# Step 5: Initialize rosdep
sudo rosdep init
rosdep update

# Step 6: Setup environment
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Step 7: Install additional build tools
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### Installing Required ROS Packages

```bash
# Navigation stack
sudo apt install ros-melodic-navigation

# SLAM packages
sudo apt install ros-melodic-slam-gmapping
sudo apt install ros-melodic-map-server

# Visualization
sudo apt install ros-melodic-rviz

# Serial communication
sudo apt install ros-melodic-rosserial-arduino
sudo apt install ros-melodic-rosserial

# Teleoperation
sudo apt install ros-melodic-teleop-twist-keyboard

# Transform library
sudo apt install ros-melodic-tf2-tools

# LIDAR drivers
sudo apt install ros-melodic-rplidar-ros

# Additional utilities
sudo apt install ros-melodic-robot-state-publisher
sudo apt install ros-melodic-joint-state-publisher
```

### Creating ROS Workspace

```bash
# Create catkin workspace
mkdir -p ~/minibot_ws/src
cd ~/minibot_ws/
catkin_make

# Source workspace
echo "source ~/minibot_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Arduino Firmware Setup

### Installing Arduino IDE and Libraries

```bash
# Download and install Arduino IDE
wget https://downloads.arduino.cc/arduino-1.8.19-linux64.tar.xz
tar -xf arduino-1.8.19-linux64.tar.xz
cd arduino-1.8.19
sudo ./install.sh
```

### Required Arduino Libraries

Install these libraries through Arduino IDE Library Manager:

1. **Rosserial Arduino Library**
   - Enables ROS communication
   - Provides ROS message types

2. **Encoder Library**
   - Hardware interrupt-based encoder reading
   - Precise position tracking

3. **LiquidCrystal I2C**
   - LCD display communication
   - System status output

### Arduino Firmware Code Structure

```cpp
// minibot_arduino_firmware.ino
#include <ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <PinChangeInt.h>
#include <string.h>  

// ==== MOTOR ENCODER AND CONTROL PINS ====
#define ENC_A1  2     
#define ENC_B1  8     

#define ENC_A2  3     
#define ENC_B2  7     

#define M1A     9
#define M1B     10

#define M2A     5
#define M2B     6

#define COUNTS_PER_REV 246.0

// ==== Encoder Variables ====
volatile long encoderPos1 = 0;
volatile long encoderPos2 = 0;
volatile int dir1 = 1;
volatile int dir2 = 1;

// ==== Motor PWM Values ====
int pwm1 = 0;
int pwm2 = 0;

unsigned long lastTime = 0;

// ==== ROS Node Handle ====
ros::NodeHandle nh;

// ==== ROS Messages ====
sensor_msgs::JointState joint_state_msg;
char* joint_names[] = {"left_wheel_joint", "right_wheel_joint"};
float positions[2];
float velocities[2];

// ==== ROS Publisher ====
ros::Publisher joint_pub("joint_states", &joint_state_msg);

// ==== ROS Subscriber Callbacks ====
void pwm1_cb(const std_msgs::Int16 &cmd) {
  pwm1 = cmd.data;
}
void pwm2_cb(const std_msgs::Int16 &cmd) {
  pwm2 = cmd.data;
}

ros::Subscriber<std_msgs::Int16> pwm1_sub("motor1_cmd", &pwm1_cb);
ros::Subscriber<std_msgs::Int16> pwm2_sub("motor2_cmd", &pwm2_cb);

// ==== Setup ====
void setup() {
  // Encoder pins
  pinMode(ENC_A1, INPUT_PULLUP);
  pinMode(ENC_B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderISR1, FALLING);

  pinMode(ENC_A2, INPUT_PULLUP);
  pinMode(ENC_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderISR2, FALLING);

  // Motor control pins
  pinMode(M1A, OUTPUT);
  pinMode(M1B, OUTPUT);
  pinMode(M2A, OUTPUT);
  pinMode(M2B, OUTPUT);

  // ROS Setup
  nh.initNode();
  nh.subscribe(pwm1_sub);
  nh.subscribe(pwm2_sub);
  nh.advertise(joint_pub);

  // Joint State Message setup
  joint_state_msg.name_length = 2;
  joint_state_msg.name = joint_names;
  joint_state_msg.position_length = 2;
  joint_state_msg.position = positions;
  joint_state_msg.velocity_length = 2;
  joint_state_msg.velocity = velocities;
}

// ==== Loop ====
void loop() {
  unsigned long now = millis();

  // Set motor speeds
  pwmOut(M1A, M1B, pwm1);
  pwmOut(M2A, M2B, pwm2);

  // Publish joint states every 100ms
  if (now - lastTime >= 100) {
    joint_state_msg.header.stamp = nh.now();

    joint_state_msg.position[0] = (2.0 * 3.1416 * encoderPos1) / COUNTS_PER_REV;
    joint_state_msg.position[1] = (2.0 * 3.1416 * encoderPos2) / COUNTS_PER_REV;

    joint_state_msg.velocity[0] = 0;  // Optional: implement real velocity
    joint_state_msg.velocity[1] = 0;

    joint_pub.publish(&joint_state_msg);
    lastTime = now;
  }

  nh.spinOnce();
  delay(10);
}

// ==== PWM Motor Output ====
void pwmOut(int pinA, int pinB, int out) {
  out = constrain(out, -255, 255);
  if (out >= 0) {
    analogWrite(pinA, 0);
    analogWrite(pinB, out);
  } else {
    analogWrite(pinA, -out);
    analogWrite(pinB, 0);
  }
}

// ==== Encoder Interrupt Service Routines (ISR) ====
void encoderISR1() {
  // ENC_B1 is on pin 8 → Port B bit 0 (PB0)
  if (PINB & (1 << 0)) {
    encoderPos1++;
    dir1 = 1;
  } else {
    encoderPos1--;
    dir1 = -1;
  }
}

void encoderISR2() {
  // ENC_B2 is on pin 7 → Port D bit 7 (PD7)
  if (PIND & (1 << 7)) {
    encoderPos2++;
    dir2 = 1;
  } else {
    encoderPos2--;
    dir2 = -1;
  }
}

```

### Uploading Firmware

1. **Connect Arduino to Your PC** via USB
2. **Select correct board and port** in Arduino IDE
3. **Compile and upload** the firmware
4. **Verify communication** using Serial Monitor

---

## ROS Integration

### Setting up rosserial Communication

Create a launch file for rosserial communication:

```xml
<!-- controller.launch -->
<launch>
  <!-- Load robot description -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find mobile_robot)/urdf/my_robo_hardware.urdf'" />

  <!-- Publish joint states -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />

  <!-- Load controller params -->
  <rosparam file="$(find mobile_robot)/config/controllers.yaml" command="load" />

  <!-- Hardware interface node -->
  <node name="hardware_interface" pkg="mobile_robot" type="diff_drive_hardware_interface" output="screen" />

  <!-- Spawner for joint state controller -->
  <node name="joint_state_spawner" pkg="controller_manager" type="spawner"
        args="joint_state_controller" output="screen" />

  <!-- Spawner for diff drive controller -->
  <node name="diff_drive_spawner" pkg="controller_manager" type="spawner"
        args="diff_drive_controller" output="screen">
 
  </node>
</launch>
```

### Creating Robot Description (URDF)

```xml
<!-- my_robo_hardware.urdf -->
<?xml version="1.0"?>	
<robot name="robot">

  <material name="Orange">
    <color rgba="0.5 0.2 0.0 1" />
  </material>
  <material name="Yellow">
    <color rgba="0.5 0.5 0.0 1" />
  </material>
  <material name="Black">
    <color rgba="0.5 0.0 0.0 1" />
  </material>
  <material name="Blue">
    <color rgba="0.0 0.0 0.4 1" />
  </material>
  <material name="Green">
    <color rgba="0.0 0.4 0.0 1" />
  </material>
  <material name="White">
    <color rgba="1 1 1 1" />
  </material>

  <link name="base_footprint" />

  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="70.0" />
      <inertia ixx="0.079945" ixy="0.00012078" ixz="0.00015606" iyy="0.068406" iyz="-0.0049053" izz="0.12343" />
    </inertial>
  <visual>
    <origin xyz="0 0 -0.015" rpy="0 0 -1.57" />
    <geometry>
      <mesh filename="package://mobile_robot/urdf/meshes/Base_Plate.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="Blue" />
  </visual>
  <collision>
    <origin xyz="0 0 -0.015" rpy="0 0 -1.57" />
    <geometry>
      <mesh filename="package://mobile_robot/urdf/meshes/Base_Plate.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
  </link>

  <joint name="base_joint" type="fixed">
    <origin xyz="0 0 0.05335" rpy="0 0 0" />
    <parent link="base_footprint" />
    <child link="base_link" />
    <axis xyz="0 0 0" />
  </joint>

  <link name="robot_body_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="70.0" />
      <inertia ixx="0.079945" ixy="0.00012078" ixz="0.00015606" iyy="0.068406" iyz="-0.0049053" izz="0.12343" />
    </inertial>
  <visual>
    <origin xyz="0.0 0 -0.19125" rpy="0 0 -1.57" />
    <geometry>
      <mesh filename="package://mobile_robot/urdf/meshes/Robot_Cover_Full.stl" scale="0.001 0.001 0.001"/>
    </geometry>
    <material name="Yellow" />
  </visual>
  <collision>
    <origin xyz="0.0 0 -0.19125" rpy="0 0 -1.57" />
    <geometry>
      <mesh filename="package://mobile_robot/urdf/meshes/Robot_Cover_Full.stl" scale="0.001 0.001 0.001"/>
    </geometry>
  </collision>
  </link>

  <joint name="robot_body_joint" type="fixed">
    <origin xyz="0 0 0.175" rpy="0 0 0" />
    <parent link="base_link" />
    <child link="robot_body_link" />
    <axis xyz="0 0 0" />
  </joint>

  <link name="right_wheel_link">
    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
      <mass value="0.50917" />
      <inertia ixx="0.0012185" ixy="1.8059E-20" ixz="3.0427E-20" iyy="0.0012185" iyz="-4.0491E-20" izz="0.0022928" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.03" radius="0.04" />
      </geometry>
      <material name="Black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.03" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="right_wheel_joint" type="continuous">
    <origin xyz="0.0 -0.12 0.01" rpy="1.571 0 0" />
    <parent link="base_link" />
    <child link="right_wheel_link" />
    <axis xyz="0 0 -1" />
  </joint>

  <link name="left_wheel_link">
    <inertial>
      <origin xyz="0.0 0.0 0.0" rpy="0 0 0" />
      <mass value="0.50917" />
      <inertia ixx="0.0012185" ixy="1.8059E-20" ixz="3.0427E-20" iyy="0.0012185" iyz="-4.0491E-20" izz="0.0022928" />
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.03" radius="0.04" />
      </geometry>
      <material name="Black" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.03" radius="0.04" />
      </geometry>
    </collision>
  </link>

  <joint name="left_wheel_joint" type="continuous">
    <origin xyz="0.0 0.12 0.01" rpy="-1.571 0 0" />
    <parent link="base_link" />
    <child link="left_wheel_link" />
    <axis xyz="0 0 1" />
  </joint>

  <link name="lidar_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="0.0093378" />
      <inertia ixx="0" ixy="0" ixz="0" iyy="0" iyz="0" izz="0" /> 
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.046" radius="0.035" />
      </geometry>
      <material name="Green" />
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder length="0.046" radius="0.035" />
      </geometry>
    </collision>
  </link>

  <joint name="laser_joint" type="fixed">
    <origin xyz="0.0075 0.0 0.35" rpy="0 0 3.14" />
    <parent link="base_link" />
    <child link="lidar_link" />
    <axis xyz="0 0 0" />
  </joint>

  <!-- Transmission (for ros_control) -->
  <transmission name="left_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="left_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="left_wheel_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>

  <transmission name="right_wheel_trans">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="right_wheel_joint">
      <hardwareInterface>hardware_interface/VelocityJointInterface</hardwareInterface>
    </joint>
    <actuator name="right_wheel_motor"><mechanicalReduction>1</mechanicalReduction></actuator>
  </transmission>

</robot>
```
</robot>
```

### Custom ROS Nodes

#### Hardware Interface (C++)

```cpp
// File: src/diff_drive_hardware_interface.cpp

#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

class DiffDriveHardware : public hardware_interface::RobotHW {
public:
  DiffDriveHardware(ros::NodeHandle& nh) : nh_(nh) {
    for (int i = 0; i < 2; ++i) {
      pos_[i] = 0.0;
      vel_[i] = 0.0;
      eff_[i] = 0.0;
      cmd_[i] = 0.0;
      smoothed_cmd_[i] = 0.0;
    }

    // Register interfaces
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle("left_wheel_joint", &pos_[0], &vel_[0], &eff_[0]));
    joint_state_interface_.registerHandle(hardware_interface::JointStateHandle("right_wheel_joint", &pos_[1], &vel_[1], &eff_[1]));
    registerInterface(&joint_state_interface_);

    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle("left_wheel_joint"), &cmd_[0]));
    velocity_joint_interface_.registerHandle(hardware_interface::JointHandle(joint_state_interface_.getHandle("right_wheel_joint"), &cmd_[1]));
    registerInterface(&velocity_joint_interface_);

    // ROS communication
    joint_state_sub_ = nh_.subscribe("/joint_states", 10, &DiffDriveHardware::jointStateCallback, this);
    motor1_pub_ = nh_.advertise<std_msgs::Int16>("/motor1_cmd", 10);
    motor2_pub_ = nh_.advertise<std_msgs::Int16>("/motor2_cmd", 10);
  }

  void read() {
    // Data updated from callback
  }

  void write() {
    std_msgs::Int16 msg1, msg2;
    const double velocity_to_pwm = 100.0;
    const double alpha = 0.7;  // smoothing factor
    const double deadband = 0.05;

    // Apply deadband
    for (int i = 0; i < 2; ++i) {
      if (std::abs(cmd_[i]) < deadband) cmd_[i] = 0.0;
    }

    // Apply smoothing
    smoothed_cmd_[0] = alpha * smoothed_cmd_[0] + (1.0 - alpha) * cmd_[0];
    smoothed_cmd_[1] = alpha * smoothed_cmd_[1] + (1.0 - alpha) * cmd_[1];

    int pwm1 = static_cast<int>(smoothed_cmd_[0] * velocity_to_pwm);
    int pwm2 = static_cast<int>(smoothed_cmd_[1] * velocity_to_pwm);

    // Clamp to PWM range
    pwm1 = std::max(-200, std::min(200, pwm1));
    pwm2 = std::max(-200, std::min(200, pwm2));

    msg1.data = pwm1;
    msg2.data = pwm2;

    motor1_pub_.publish(msg1);
    motor2_pub_.publish(msg2);
  }

  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (msg->position.size() >= 2) {
      pos_[0] = msg->position[0];
      pos_[1] = msg->position[1];
    }
    if (msg->velocity.size() >= 2) {
      vel_[0] = msg->velocity[0];
      vel_[1] = msg->velocity[1];
    }
  }

private:
  ros::NodeHandle nh_;
  hardware_interface::JointStateInterface joint_state_interface_;
  hardware_interface::VelocityJointInterface velocity_joint_interface_;

  double pos_[2];
  double vel_[2];
  double eff_[2];
  double cmd_[2];
  double smoothed_cmd_[2];

  ros::Subscriber joint_state_sub_;
  ros::Publisher motor1_pub_;
  ros::Publisher motor2_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "diff_drive_hardware_interface");
  ros::NodeHandle nh;
  DiffDriveHardware robot(nh);
  controller_manager::ControllerManager cm(&robot, nh);

  ros::Duration(1.0).sleep();  // Let publishers/subscribers settle

  // Warmup: publish zero PWM to avoid jerks
  for (int i = 0; i < 5; ++i) {
    robot.write();
    ros::Duration(0.1).sleep();
  }

  ros::Rate rate(10);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  while (ros::ok()) {
    robot.read();
    cm.update(ros::Time::now(), ros::Duration(0.1));
    robot.write();
    rate.sleep();
  }

  return 0;
}
```

---

## LIDAR Integration

### RPLIDAR ROS Package Setup

```bash
# Clone RPLIDAR ROS package
cd ~/minibot_ws/src
git clone https://github.com/Slamtec/rplidar_ros.git

# Build the package
cd ~/minibot_ws
catkin_make

# Source the workspace
source devel/setup.bash
```

### LIDAR Launch Configuration

```xml
<!-- minibot_lidar.launch -->
<launch>
    <!-- RPLIDAR A1 -->
    <node name="rplidarNode" pkg="rplidar_ros" type="rplidarNode" output="screen">
        <param name="serial_port" type="string" value="/dev/ttyUSB0"/>
        <param name="serial_baudrate" type="int" value="115200"/>
        <param name="frame_id" type="string" value="laser"/>
        <param name="inverted" type="bool" value="false"/>
        <param name="angle_compensate" type="bool" value="true"/>
    </node>
    
    <!-- Static transform from base_link to laser -->
    <node pkg="tf" type="static_transform_publisher" name="base_link_to_laser"
          args="0.1 0 0.1 0 0 0 base_link laser 50" />
</launch>
```

### Testing LIDAR Connection

```bash
# Test LIDAR connectivity
ls -la /dev/ttyUSB*

# Launch LIDAR node
roslaunch minibot_bringup minibot_lidar.launch

# Verify LIDAR data in another terminal
rostopic echo /scan

# Check transform tree
rosrun tf view_frames
evince frames.pdf
```

### LIDAR Data Visualization

```bash
# Launch RViz with LIDAR visualization
rosrun rviz rviz

# In RViz:
# 1. Set Fixed Frame to "laser"
# 2. Add LaserScan display
# 3. Set Topic to "/scan"
# 4. Adjust settings for better visualization
```

---

## Navigation Stack Setup

### Creating Navigation Launch Files

#### Base Navigation Launch

```xml
<!-- Navigation with move_base -->
<launch>
    <!-- Map server (if using pre-built map) -->
    <arg name="map_file" default="$(find mobile_robot)/map/map.yaml"/>
    <node name="map_server" pkg="map_server" type="map_server" args="$(arg map_file)" />
    
    <!-- AMCL Localization -->
    <include file="$(find mobile_robot)/launch/amcl.launch"/>
    
    <!-- Move Base -->
    <include file="$(find navigation)/launch/move_base.launch"/>
    
    <!-- Transform Configuration -->
    <node pkg="tf" type="static_transform_publisher" name="map_to_odom" 
          args="0 0 0 0 0 0 map odom 100" />
</launch>
```

#### AMCL Configuration

```xml
<!-- amcl.launch -->
<launch>
    <node pkg="amcl" type="amcl" name="amcl" output="screen">
        <!-- Overall filter parameters -->
        <param name="min_particles" value="500"/>
        <param name="max_particles" value="3000"/>
        <param name="kld_err" value="0.05"/>
        <param name="kld_z" value="0.99"/>
        <param name="update_min_d" value="0.2"/>
        <param name="update_min_a" value="0.5"/>
        <param name="resample_interval" value="1"/>
        <param name="transform_tolerance" value="0.1"/>
        <param name="recovery_alpha_slow" value="0.001"/>
        <param name="recovery_alpha_fast" value="0.1"/>
        
        <!-- Laser model parameters -->
        <param name="laser_max_beams" value="60"/>
        <param name="laser_z_hit" value="0.5"/>
        <param name="laser_z_short" value="0.05"/>
        <param name="laser_z_max" value="0.05"/>
        <param name="laser_z_rand" value="0.5"/>
        <param name="laser_sigma_hit" value="0.2"/>
        <param name="laser_lambda_short" value="0.1"/>
        <param name="laser_model_type" value="likelihood_field"/>
        
        <!-- Odometry model parameters -->
        <param name="odom_model_type" value="diff"/>
        <param name="odom_alpha1" value="0.2"/>
        <param name="odom_alpha2" value="0.2"/>
        <param name="odom_alpha3" value="0.8"/>
        <param name="odom_alpha4" value="0.2"/>
        <param name="odom_frame_id" value="odom"/>
        <param name="base_frame_id" value="base_link"/>
        <param name="global_frame_id" value="map"/>
    </node>
</launch>
```

#### Move Base Configuration

```xml
<!-- move_base.launch -->
<launch>
   <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <rosparam file="$(find navigation)/config/costmap_common_params.yaml" command="load" ns="global_costmap" /> 
    <rosparam file="$(find navigation)/config/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find navigation)/config/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find navigation)/config/global_costmap_params.yaml" command="load" /> 
    <rosparam file="$(find navigation)/config/base_local_planner_params.yaml" command="load" />
 </node>
</launch> 
<!-- Reff : http://wiki.ros.org/navigation/Tutorials/RobotSetup -->
```

### Navigation Parameter Files

#### Controllers Configuration

```yaml
# controllers.yaml
diff_drive_controller:
  type: "diff_drive_controller/DiffDriveController"
  left_wheel: 'left_wheel_joint'
  right_wheel: 'right_wheel_joint'
  publish_rate: 50
  pose_covariance_diagonal: [0.001, 0.001, 0.001, 0.0, 0.0, 0.03]
  twist_covariance_diagonal: [0.001, 0.001, 0.001, 0.0, 0.0, 0.03]

  wheel_separation: 0.30     # Adjust as per your robot
  wheel_radius: 0.05         # Adjust as per your wheel

  cmd_vel_timeout: 0.25
  enable_odom_tf: true

  base_frame_id: base_footprint
  odom_frame_id: odom

  use_stamped_vel: false
  
joint_state_controller:
  type: joint_state_controller/JointStateController
  publish_rate: 50
```

#### Costmap Common Parameters

```yaml
# costmap_common_params.yaml
obstacle_range: 2.5
raytrace_range: 3.0
footprint: [[-0.15, -0.1], [-0.15, 0.1], [0.15, 0.1], [0.15, -0.1]]
inflation_radius: 0.25
cost_scaling_factor: 3.0

observation_sources: laser_scan_sensor

laser_scan_sensor:
  sensor_frame: laser
  data_type: LaserScan
  topic: scan
  marking: true
  clearing: true
  min_obstacle_height: 0.0
  max_obstacle_height: 2.0
```

#### Local Costmap Parameters

```yaml
# local_costmap_params.yaml
local_costmap:
  global_frame: odom
  robot_base_frame: base_link
  update_frequency: 5.0
  publish_frequency: 2.0
  static_map: false
  rolling_window: true
  width: 4.0
  height: 4.0
  resolution: 0.05
  transform_tolerance: 0.5
```

#### Global Costmap Parameters

```yaml
# global_costmap_params.yaml
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  update_frequency: 1.0
  static_map: true
  transform_tolerance: 0.5
```

#### Base Local Planner Parameters

```yaml
# base_local_planner_params.yaml
TrajectoryPlannerROS:
  # Robot Configuration
  max_vel_x: 0.5
  min_vel_x: 0.1
  max_vel_theta: 1.0
  min_vel_theta: -1.0
  min_in_place_vel_theta: 0.4
  
  acc_lim_theta: 3.2
  acc_lim_x: 2.5
  acc_lim_y: 2.5
  
  # Goal Tolerance
  yaw_goal_tolerance: 0.1
  xy_goal_tolerance: 0.2
  latch_xy_goal_tolerance: false
  
  # Forward Simulation
  sim_time: 1.7
  sim_granularity: 0.025
  angular_sim_granularity: 0.1
  vx_samples: 6
  vtheta_samples: 20
  controller_frequency: 20.0
  
  # Trajectory Scoring
  meter_scoring: true
  pdist_scale: 0.6
  gdist_scale: 0.8
  occdist_scale: 0.01
  heading_lookahead: 0.325
  heading_scoring: false
  heading_scoring_timestep: 0.8
  dwa: true
  global_frame_id: odom
  
  # Oscillation Prevention
  oscillation_reset_dist: 0.25
  escape_reset_dist: 0.1
  escape_reset_theta: 0.1
```

---

## SLAM Configuration

### Setting up gmapping for SLAM

#### SLAM Launch File

```xml
<!-- minibot_slam.launch -->
<launch>
    <!-- Gmapping SLAM -->
    <node pkg="gmapping" type="slam_gmapping" name="slam_gmapping" output="screen">
        <!-- Frame names -->
        <param name="map_frame" value="map"/>
        <param name="odom_frame" value="odom"/>
        <param name="base_frame" value="base_link"/>
        
        <!-- Sensor parameters -->
        <param name="laser_frame" value="laser"/>
        <param name="sensor_range" value="4.0"/>
        
        <!-- Processing parameters -->
        <param name="map_update_interval" value="2.0"/>
        <param name="maxUrange" value="3.5"/>
        <param name="maxRange" value="4.0"/>
        <param name="sigma" value="0.05"/>
        <param name="kernelSize" value="1"/>
        <param name="lstep" value="0.05"/>
        <param name="astep" value="0.05"/>
        <param name="iterations" value="5"/>
        <param name="lsigma" value="0.075"/>
        <param name="ogain" value="3.0"/>
        <param name="lskip" value="0"/>
        
        <!-- Motion model parameters -->
        <param name="srr" value="0.01"/>
        <param name="srt" value="0.02"/>
        <param name="str" value="0.01"/>
        <param name="stt" value="0.02"/>
        
        <!-- Map parameters -->
        <param name="linearUpdate" value="0.2"/>
        <param name="angularUpdate" value="0.1"/>
        <param name="temporalUpdate" value="0.5"/>
        <param name="resampleThreshold" value="0.5"/>
        <param name="particles" value="100"/>
        
        <!-- Initial map size -->
        <param name="xmin" value="-10.0"/>
        <param name="ymin" value="-10.0"/>
        <param name="xmax" value="10.0"/>
        <param name="ymax" value="10.0"/>
        <param name="delta" value="0.05"/>
        <param name="llsamplerange" value="0.01"/>
        <param name="llsamplestep" value="0.01"/>
        <param name="lasamplerange" value="0.005"/>
        <param name="lasamplestep" value="0.005"/>
    </node>
    
    <!-- RViz for visualization -->
    <node name="rviz" pkg="rviz" type="rviz" 
          args="-d $(find minibot_navigation)/rviz/slam.rviz"/>
</launch>
```

### Complete System Launch File

```xml
<!-- minibot_complete.launch -->
<launch>
    <!-- Hardware Interface -->
    <include file="$(find minibot_bringup)/launch/minibot_serial.launch"/>
    
    <!-- LIDAR -->
    <include file="$(find minibot_bringup)/launch/minibot_lidar.launch"/>
    
    <!-- SLAM -->
    <include file="$(find minibot_navigation)/launch/minibot_slam.launch"/>
    
    <!-- Teleoperation (optional) -->
    <node name="teleop_twist_keyboard" pkg="teleop_twist_keyboard" type="teleop_twist_keyboard.py"
          output="screen" launch-prefix="xterm -e"/>
</launch>
```

### RViz Configuration for SLAM

Create an RViz config file (`slam.rviz`) with these displays:

1. **Map Display**
   - Topic: `/map`
   - Color Scheme: map

2. **LaserScan Display**
   - Topic: `/scan`
   - Size: 0.05
   - Style: Points

3. **RobotModel Display**
   - Description Source: Topic
   - Description Topic: `/robot_description`

4. **TF Display**
   - Show Names: true
   - Show Axes: true

5. **Path Display** (for planned paths)
   - Topic: `/move_base/NavfnROS/plan`

### Saving Maps

```bash
# Save the generated map
rosrun map_server map_saver -f ~/minibot_ws/src/minibot_navigation/maps/my_map

# This creates two files:
# - my_map.yaml (map metadata)
# - my_map.pgm (map image)
```

---

## Testing & Debugging

### System Verification Checklist

#### Hardware Tests

```bash
# 1. Check all connections
sudo dmesg | grep tty    # Check Arduino connection
lsusb                    # Check USB devices (LIDAR, Arduino)

# 2. Test motor control manually
rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}" --once

# 3. Verify encoder feedback
rostopic echo /odom

# 4. Test LIDAR data
rostopic echo /scan | head -20
```

#### ROS Communication Tests

```bash
# Check ROS master
roscore &
rosnode list

# Verify topics
rostopic list
rostopic info /cmd_vel
rostopic info /scan
rostopic info /odom

# Check transforms
rosrun tf view_frames
evince frames.pdf

# Monitor transform tree
rosrun tf tf_monitor
```

#### Navigation System Tests

```bash
# 1. Test localization
roslaunch minibot_navigation minibot_navigation.launch

# 2. Send navigation goals via RViz
# Click "2D Nav Goal" and set target

# 3. Monitor navigation performance
rostopic echo /move_base/status
rostopic echo /move_base/feedback
```

### Common Issues and Solutions

#### Problem: Arduino Not Connecting

**Symptoms:**
- `rosserial_python` shows connection errors
- No `/odom` topic published

**Solutions:**
```bash
# Check USB permissions
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0

# Reset Arduino connection
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600

# Verify baud rate matches Arduino code
```

#### Problem: LIDAR Not Detected

**Symptoms:**
- `/scan` topic not available
- RPLIDAR node fails to start

**Solutions:**
```bash
# Check LIDAR connection
ls -la /dev/ttyUSB*

# Test LIDAR directly
sudo chmod 666 /dev/ttyUSB0
roslaunch rplidar_ros rplidar.launch

# Verify frame configuration
rostopic echo /scan | grep frame_id
```

#### Problem: Navigation Not Working

**Symptoms:**
- Robot doesn't move to goal
- Path planning fails

**Solutions:**
```bash
# Check costmap configuration
rostopic echo /move_base/global_costmap/costmap
rostopic echo /move_base/local_costmap/costmap

# Verify transform chain
rosrun tf tf_echo map base_link

# Adjust navigation parameters
rosparam set /move_base/TrajectoryPlannerROS/max_vel_x 0.3
```

#### Problem: Poor SLAM Performance

**Symptoms:**
- Map quality is poor
- Localization drifts

**Solutions:**
```bash
# Increase particle count
rosparam set /slam_gmapping/particles 200

# Adjust motion model
rosparam set /slam_gmapping/srr 0.005
rosparam set /slam_gmapping/stt 0.01

# Ensure adequate motion during mapping
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```


