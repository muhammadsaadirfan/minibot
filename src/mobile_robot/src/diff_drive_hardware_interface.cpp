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
