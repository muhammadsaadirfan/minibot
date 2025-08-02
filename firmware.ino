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
