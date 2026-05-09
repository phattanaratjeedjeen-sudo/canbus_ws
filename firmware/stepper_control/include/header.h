// base
#include <Arduino.h>
#include <micro_ros_platformio.h>

// gripper
#include <ESP32Servo.h>

// stepper
#include <Wire.h>
#include <math.h>
#include "driver/gpio.h"
#include "MotorController.h"

// ROS2
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ROS2 messages
#include <std_msgs/msg/u_int16_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>