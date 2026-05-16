// base 
#include <Arduino.h>
#include <math.h>

// magnetic encoder
#include <Wire.h>

// gripper
#include <ESP32Servo.h>

// micro ros
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

// micro ros messages
#include <std_msgs/msg/u_int16_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>

#define AS5600_SDA1 21  
#define AS5600_SCL1 22  
#define TB6600_DIR1 27  
#define TB6600_PUL1 13  

#define AS5600_SDA2 16
#define AS5600_SCL2 17 
#define TB6600_DIR2 33
#define TB6600_PUL2 32

#define AS5600_ADDR 0x36
#define RAW_ANGLE_MSB_REG 0x0C

hw_timer_t * stepTimer1 = NULL;
hw_timer_t * stepTimer2 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;
float_t setSpeed1 = 0.0;
float_t setSpeed2 = 0.0;

# define SERVO_PIN1 18
# define SERVO_PIN2 19

Servo servo1;
Servo servo2;

rcl_publisher_t publisher;
sensor_msgs__msg__JointState step_joint_feedback;

rcl_subscription_t step_joint_subscriber;
sensor_msgs__msg__JointState step_joint_cmd;
rcl_subscription_t gripper_subscriber;
std_msgs__msg__UInt16MultiArray gripper_cmd;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t publishTimer; 
rcl_timer_t controlTimer; 

struct MotorDebugData {
    float_t targetSpeed;
    float_t filteredSpeed;
    float_t commandSpeed;
    float_t dt;
    float_t numRevolutions;
};

MotorDebugData debugData1, debugData2;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void setup();
void error_loop();
void publishTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void controlTimerCallback(rcl_timer_t * timer, int64_t last_call_time);
void gripper_cmd_callback(const void * msgin);
void step_joint_cmd_callback(const void * msgin);
void grip(int16_t angle1, int16_t angle2);

void IRAM_ATTR onStepTimer1();
void IRAM_ATTR onStepTimer2();

void gripperSetup();
void motorSetup();
void ISRSetup();

void error_loop() {
  while(1) {
    delay(100);
  }
}

void gripperSetup() {
  uint16_t minUs = 500; 
  uint16_t maxUs = 2500;
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo1.setPeriodHertz(50);
  servo2.setPeriodHertz(50);
  servo1.attach(SERVO_PIN1, minUs, maxUs);
  servo2.attach(SERVO_PIN2, minUs, maxUs);
}

void motorSetup() {
  Wire.begin(AS5600_SDA1, AS5600_SCL1);
  Wire.setClock(400000);
  pinMode(TB6600_DIR1, OUTPUT);
  pinMode(TB6600_PUL1, OUTPUT);
  digitalWrite(TB6600_DIR1, LOW);

  Wire1.begin(AS5600_SDA2, AS5600_SCL2);
  Wire1.setClock(400000);
  pinMode(TB6600_DIR2, OUTPUT);
  pinMode(TB6600_PUL2, OUTPUT);
  digitalWrite(TB6600_DIR2, LOW);
}

void ISRSetup() {
  stepTimer1 = timerBegin(0, 80, true); 
  timerAttachInterrupt(stepTimer1, &onStepTimer1, true);
  timerAlarmEnable(stepTimer1);

  stepTimer2 = timerBegin(1, 80, true); 
  timerAttachInterrupt(stepTimer2, &onStepTimer2, true);
  timerAlarmEnable(stepTimer2);
}

void setup() {
  gripperSetup();
  motorSetup();

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


  rosidl_runtime_c__String__Sequence__init(&step_joint_feedback.name, 2);
  rosidl_runtime_c__String__assign(&step_joint_feedback.name.data[0], "motor1");
  rosidl_runtime_c__String__assign(&step_joint_feedback.name.data[1], "motor2");
  rosidl_runtime_c__double__Sequence__init(&step_joint_feedback.position, 2);
  rosidl_runtime_c__double__Sequence__init(&step_joint_feedback.velocity, 2);
  
  rosidl_runtime_c__String__Sequence__init(&step_joint_cmd.name, 2);
  rosidl_runtime_c__double__Sequence__init(&step_joint_cmd.position, 2);
  rosidl_runtime_c__double__Sequence__init(&step_joint_cmd.velocity, 2);
  
  rosidl_runtime_c__uint16__Sequence__init(&gripper_cmd.data, 2);

  // create timer,
  const uint32_t publishFreq = 50;
  RCCHECK(rclc_timer_init_default2(
    &publishTimer,
    &support,
    RCL_MS_TO_NS(1000 / publishFreq),
    publishTimerCallback,
    true));

  const uint32_t controlFreq = 200;
  RCCHECK(rclc_timer_init_default2(
    &controlTimer,
    &support,
    RCL_MS_TO_NS(1000 / controlFreq),
    controlTimerCallback,
    true));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &step_joint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "step_joint_command"));
  
  RCCHECK(rclc_subscription_init_default(
    &gripper_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "gripper_command"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &publishTimer));
  RCCHECK(rclc_executor_add_timer(&executor, &controlTimer));
  RCCHECK(rclc_executor_add_subscription(&executor, &step_joint_subscriber, &step_joint_cmd, step_joint_cmd_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &gripper_subscriber, &gripper_cmd, gripper_cmd_callback, ON_NEW_DATA));
  
  ISRSetup();
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50)));
}

void publishTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    step_joint_feedback.position.data[0] = debugData1.numRevolutions; 
    step_joint_feedback.position.data[1] = debugData2.numRevolutions;
    step_joint_feedback.velocity.data[0] = debugData1.filteredSpeed;
    step_joint_feedback.velocity.data[1] = debugData2.filteredSpeed;
    RCSOFTCHECK(rcl_publish(&publisher, &step_joint_feedback, NULL));
  }
}

// -------------------------------- motor control class --------------------------------
class StepperController {
private:
    uint8_t motorID;
    TwoWire* wire;
    uint8_t pulPin;
    uint8_t dirPin;
    hw_timer_t** timer;
    portMUX_TYPE* mux;
    MotorDebugData* debugData;
    
    float_t previousRadAngle = 0.0;
    float_t totalRadAngle = 0.0;
    float_t previousFilteredSpeed = 0.0;
    unsigned long lastTime = 0;
    float_t targetSpeed = 0.0;
    float_t integral = 0.0;
    float_t previousError = 0.0;
    boolean isTimerRunning = true;
    
    float_t cutoffFreq = 5.0;
    float_t accelation = 20.0;
    float_t Kp = 0.3;
    float_t Ki = 0.2;
    float_t Kd = 0.01;

    uint16_t readRawAngle() {
        uint16_t rawAngle = 0;
        wire->beginTransmission(AS5600_ADDR);
        wire->write(RAW_ANGLE_MSB_REG);
        wire->endTransmission(false);
        wire->requestFrom(AS5600_ADDR, 2);
        if (wire->available() >= 2) {
            uint8_t msb = wire->read();
            uint8_t lsb = wire->read();
            rawAngle = (msb << 8) | lsb;
        }
        return rawAngle;
    }

    void setStepperSpeed(uint32_t usInterval) {
        portENTER_CRITICAL(mux);
        timerAlarmWrite(*timer, usInterval, true);
        portEXIT_CRITICAL(mux);
    }

public:
    StepperController(uint8_t id, TwoWire& w, uint8_t pPin, uint8_t dPin, hw_timer_t** tmr, portMUX_TYPE* mx, MotorDebugData* dbgData) 
        : motorID(id), wire(&w), pulPin(pPin), dirPin(dPin), timer(tmr), mux(mx), debugData(dbgData) {}

    void control(float_t setSpeed) {
        float_t radAngle = (readRawAngle() / 4096.0) * 2.0 * PI;
        float_t deltaAngle = remainder(radAngle - previousRadAngle, 2.0 * PI);
        totalRadAngle += deltaAngle;
        previousRadAngle = radAngle;

        float_t tau = 1.0 / (2.0 * PI * cutoffFreq);
        unsigned long now = micros();
        float_t dt = (now - lastTime) / 1000000.0;
        if (dt <= 0.0) dt = 0.0001; 
        lastTime = now;
        
        float_t alpha = dt / (tau + dt);
        float_t speed = deltaAngle / dt;
        float_t filteredSpeed = previousFilteredSpeed + alpha * (speed - previousFilteredSpeed);
        previousFilteredSpeed = filteredSpeed;

        float_t deltaSpeed = accelation * dt;
        float_t diffSpeed = setSpeed - targetSpeed;
        float_t changedSpeed = fmaxf(-deltaSpeed, fminf(diffSpeed, deltaSpeed));
        targetSpeed += changedSpeed;

        if (abs(setSpeed) < 0.01 && abs(targetSpeed) < 0.05) {
            targetSpeed = 0.0;
        }

        float_t error = targetSpeed - filteredSpeed;
        
        if (targetSpeed == 0.0) {
            integral = 0.0;
        } else {
            integral += (error * dt);
            integral = fmaxf(fminf(integral, 50.0), -50.0);
        }
        
        float_t derivative = (error - previousError) / dt;
        previousError = error;

        float_t outputSpeed = (Kp * error) + (Ki * integral) + (Kd * derivative);
        outputSpeed = fmaxf(fminf(outputSpeed, 4.0), -4.0);
        
        float_t commandSpeed = fmaxf(fminf(targetSpeed + outputSpeed, 100.0), -100.0);

        if (abs(commandSpeed) <= 0.1) {
            if (isTimerRunning) {
                timerAlarmDisable(*timer);
                isTimerRunning = false;
            }
            return;
        } else {
            if (!isTimerRunning) {
                timerAlarmEnable(*timer);
                isTimerRunning = true;
            }
        }

        uint32_t stepFreq = (uint32_t)(abs(commandSpeed) * 1600.0 / (2 * PI));
        if (stepFreq == 0) stepFreq = 1;
        uint32_t usInterval = 1000000 / (stepFreq * 2);
        if (usInterval < 20) usInterval = 20;

        gpio_set_level((gpio_num_t)dirPin, commandSpeed > 0 ? 0 : 1);
        setStepperSpeed(usInterval);
        
        if (debugData) {
            debugData->targetSpeed = targetSpeed;
            debugData->filteredSpeed = filteredSpeed;
            debugData->commandSpeed = commandSpeed;
            debugData->dt = dt;
            debugData->numRevolutions = totalRadAngle / (2.0 * PI);
        }
    }
};

StepperController motor1(1, Wire, TB6600_PUL1, TB6600_DIR1, &stepTimer1, &timerMux1, &debugData1);
StepperController motor2(2, Wire1, TB6600_PUL2, TB6600_DIR2, &stepTimer2, &timerMux2, &debugData2);

void controlTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motor1.control(setSpeed1);
    motor2.control(setSpeed2);
  }
}

// -------------------------------- gripper control --------------------------------
void gripper_cmd_callback(const void * msgin) {
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *) msgin;
  if (msg->data.size >= 2) {
    int16_t angle1 = msg->data.data[0];
    int16_t angle2 = msg->data.data[1];
    grip(angle1, angle2);
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


// -------------------------------- motor control --------------------------------
void step_joint_cmd_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *) msgin;
  if (msg->velocity.size == 2) {
    setSpeed1 = msg->velocity.data[0];
    setSpeed2 = msg->velocity.data[1];    
  } else {
    return; 
  }
}

void IRAM_ATTR onStepTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  static boolean state = 1;
  state = !state;
  gpio_set_level((gpio_num_t)TB6600_PUL1, state ? 1 : 0);
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onStepTimer2() {
  portENTER_CRITICAL_ISR(&timerMux2);
  static boolean state = 1;
  state = !state;
  gpio_set_level((gpio_num_t)TB6600_PUL2, state ? 1 : 0);
  portEXIT_CRITICAL_ISR(&timerMux2);
}
