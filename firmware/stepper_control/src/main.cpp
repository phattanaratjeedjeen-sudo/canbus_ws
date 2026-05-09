#include <Arduino.h>
#include <micro_ros_platformio.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_srvs/srv/set_bool.h>
#include <std_srvs/srv/trigger.h>

// gripper
# define SERVO_PIN1 18
# define SERVO_PIN2 19
Servo servo1;
Servo servo2;
struct {
  int16_t grip1 = 60;
  int16_t grip2 = 0;
  int16_t release1 = 200;
  int16_t release2 = 65;
}servoPositions;


rcl_publisher_t publisher;
sensor_msgs__msg__JointState step_joint_feedback;

rcl_subscription_t subscriber;
sensor_msgs__msg__JointState step_joint_cmd;

rcl_service_t service_grip;
std_srvs__srv__SetBool_Request req_grip;
std_srvs__srv__SetBool_Response res_grip;

rcl_service_t go_home;
std_srvs__srv__Trigger_Request req_go_home;
std_srvs__srv__Trigger_Response res_go_home;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void step_joint_cmd_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *) msgin;
  for (size_t i = 0; i < msg->position.size; i++) {
    step_joint_feedback.position.data[i] = msg->position.data[i];
  }
}

void grip(int16_t angle1, int16_t angle2) 
{
  int16_t maxAngle = 270; 
  angle1 = constrain(angle1, 0, maxAngle);
  angle2 = constrain(angle2, 0, maxAngle);
  int16_t pulseWidth1 = map(angle1, 0, maxAngle, 500, 2500);
  int16_t pulseWidth2 = map(angle2, 0, maxAngle, 500, 2500);
  servo1.writeMicroseconds(pulseWidth1);
  servo2.writeMicroseconds(pulseWidth2);
}

void grip_callback(const void * req, void * res) {
  std_srvs__srv__SetBool_Request * req_in = (std_srvs__srv__SetBool_Request *) req;
  std_srvs__srv__SetBool_Response * res_in = (std_srvs__srv__SetBool_Response *) res;
  
  if (req_in->data) {
    grip(servoPositions.grip1, servoPositions.grip2);
  } else {
    grip(servoPositions.release1, servoPositions.release2);
  }
  
  res_in->success = true;
}

void go_home_callback(const void * req, void * res) {
  std_srvs__srv__Trigger_Request * req_in = (std_srvs__srv__Trigger_Request *) req;
  std_srvs__srv__Trigger_Response * res_in = (std_srvs__srv__Trigger_Response *) res;
  res_in->success = true;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &step_joint_feedback, NULL));
  }
}

void setup() {
  uint16_t minUs = 500; 
  uint16_t maxUs = 2500;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO_PIN1, minUs, maxUs);
  servo2.attach(SERVO_PIN2, minUs, maxUs);

  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "stepper_control", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "step_joint_feedback"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback,
    true));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "step_joint_command"));

  // create service
  RCCHECK(rclc_service_init_default(
    &service_grip,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
    "grip"));

  RCCHECK(rclc_service_init_default(
    &go_home,
    &node,
    ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger),
    "go_home"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_service(&executor, &service_grip, &req_grip, &res_grip, grip_callback));
  RCCHECK(rclc_executor_add_service(&executor, &go_home, &req_go_home, &res_go_home, go_home_callback));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &step_joint_cmd, step_joint_cmd_callback, ON_NEW_DATA));

}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}