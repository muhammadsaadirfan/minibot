cmake_minimum_required(VERSION 3.0.2)
project(mobile_robot)

## Compile as C++11
add_compile_options(-std=c++11)

## Find catkin macros and ROS packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  hardware_interface
  controller_manager
  pluginlib
  transmission_interface
  joint_limits_interface
  std_msgs
  sensor_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp hardware_interface controller_manager
                 pluginlib transmission_interface joint_limits_interface
                 std_msgs sensor_msgs
)

## Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Executable for hardware interface
add_executable(diff_drive_hardware_interface src/diff_drive_hardware_interface.cpp)
add_dependencies(diff_drive_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(diff_drive_hardware_interface ${catkin_LIBRARIES})
