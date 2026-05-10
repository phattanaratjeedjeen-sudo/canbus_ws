#include "header.h"

#define AS5600_SDA1 21  
#define AS5600_SCL1 22  
#define TB6600_ENA1 26  
#define TB6600_DIR1 27  
#define TB6600_PUL1 14  

#define AS5600_SDA2 16
#define AS5600_SCL2 17
#define TB6600_ENA2 32  
#define TB6600_DIR2 33
#define TB6600_PUL2 15

hw_timer_t * stepTimer1 = NULL;
hw_timer_t * stepTimer2 = NULL;
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED;

// Mutex protection for shared variables accessed by multiple tasks/callbacks
SemaphoreHandle_t speedMutex = NULL;
SemaphoreHandle_t debugDataMutex = NULL;

float_t setSpeed1 = 0.0;
float_t setSpeed2 = 0.0;

// MotorDebugData struct is defined in MotorController.h
MotorDebugData debugData1 = {};
MotorDebugData debugData2 = {};

MotorController *motor1;
MotorController *motor2;

uint32_t controlFreq = 1000; // Hz
uint32_t publishFreq = 50;   // Hz

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
double step_joint_positions[2];
double step_joint_velocities[2];

rcl_subscription_t step_joint_subscriber;
sensor_msgs__msg__JointState step_joint_cmd;
double step_joint_cmd_positions[2];
double step_joint_cmd_velocities[2];

rcl_subscription_t gripper_subscriber;
std_msgs__msg__UInt16MultiArray gripper_cmd;
uint16_t gripper_cmd_data[2];

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop();
void IRAM_ATTR onStepTimer1();
void IRAM_ATTR onStepTimer2();
void controlTask(void *pvParameters);
void step_joint_cmd_callback(const void * msgin);
void gripper_cmd_callback(const void * msgin);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void grip(int16_t angle1, int16_t angle2);

void setup() {
  // Create mutexes for thread-safe access
  speedMutex = xSemaphoreCreateMutex();
  debugDataMutex = xSemaphoreCreateMutex();
  if (speedMutex == NULL || debugDataMutex == NULL) {
    Serial.println("[ERROR] Failed to create mutexes.");
    error_loop();
  }

  // Config Stepper & Motor Hardware
  Wire.begin(AS5600_SDA1, AS5600_SCL1);
  Wire.setClock(400000);
  
  Wire1.begin(AS5600_SDA2, AS5600_SCL2);
  Wire1.setClock(400000);
  esp_task_wdt_reset();

  pinMode(TB6600_ENA1, OUTPUT);
  pinMode(TB6600_DIR1, OUTPUT);
  pinMode(TB6600_PUL1, OUTPUT);
  digitalWrite(TB6600_ENA1, LOW); 
  digitalWrite(TB6600_DIR1, LOW);

  pinMode(TB6600_ENA2, OUTPUT);
  pinMode(TB6600_DIR2, OUTPUT);
  pinMode(TB6600_PUL2, OUTPUT);
  digitalWrite(TB6600_ENA2, LOW);
  digitalWrite(TB6600_DIR2, LOW);
  esp_task_wdt_reset();

  stepTimer1 = timerBegin(0, 80, true); 
  timerAttachInterrupt(stepTimer1, &onStepTimer1, true);
  timerAlarmWrite(stepTimer1, 2000, true);
  // Don't enable yet - will be enabled when motor starts
  // timerAlarmEnable(stepTimer1);

  stepTimer2 = timerBegin(1, 80, true); 
  timerAttachInterrupt(stepTimer2, &onStepTimer2, true);
  timerAlarmWrite(stepTimer2, 2000, true);
  // Don't enable yet - will be enabled when motor starts
  // timerAlarmEnable(stepTimer2);
  esp_task_wdt_reset();
  
  motor1 = new MotorController(1, Wire, TB6600_PUL1, TB6600_DIR1, stepTimer1, &timerMux1, 5.0);
  motor2 = new MotorController( 2,  Wire1,  TB6600_PUL2, TB6600_DIR2,  stepTimer2,  &timerMux2,  5.0);
  esp_task_wdt_reset();

  xTaskCreatePinnedToCore(
    controlTask,   // Task function.
    "ControlTask", // String with name of task.
    4096,          // Stack size in bytes.
    NULL,          // Parameter passed as input of the task
    5,             // Priority of the task (higher = more priority).
    NULL,          // Task handle.
    1              // Core where the task should run
  );            
  esp_task_wdt_reset();

  // Config Servos
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
  esp_task_wdt_reset();

  // Configure serial transport
  Serial.begin(115200);
  delay(500);
  Serial.println("[STARTUP] ESP32 Board Starting - micro-ROS initialization...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  Serial.println("[STARTUP] Initializing micro-ROS client...");
  esp_task_wdt_reset();

  allocator = rcl_get_default_allocator();
  esp_task_wdt_reset();

  // Pre-allocate buffers for subscription messages with dynamic arrays
  step_joint_feedback.position.data = step_joint_positions;
  step_joint_feedback.position.capacity = 2;
  step_joint_feedback.position.size = 2;

  step_joint_feedback.velocity.data = step_joint_velocities;
  step_joint_feedback.velocity.capacity = 2;
  step_joint_feedback.velocity.size = 2;
  
  step_joint_cmd.position.data = step_joint_cmd_positions;
  step_joint_cmd.position.capacity = 2;
  step_joint_cmd.position.size = 2;

  step_joint_cmd.velocity.data = step_joint_cmd_velocities;
  step_joint_cmd.velocity.capacity = 2;
  step_joint_cmd.velocity.size = 2;

  gripper_cmd.data.data = gripper_cmd_data;
  gripper_cmd.data.capacity = 2;
  gripper_cmd.data.size = 2;
  esp_task_wdt_reset();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  esp_task_wdt_reset();

  // create node
  RCCHECK(rclc_node_init_default(&node, "stepper_control", "", &support));
  esp_task_wdt_reset();

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "step_joint_feedback"));
  esp_task_wdt_reset();

  // create timer,
  RCCHECK(rclc_timer_init_default2(
    &timer,
    &support,
    RCL_MS_TO_NS(1000 / publishFreq),
    timer_callback,
    true));
  esp_task_wdt_reset();

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &step_joint_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "step_joint_command"));
  esp_task_wdt_reset();
  
  RCCHECK(rclc_subscription_init_default(
    &gripper_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt16MultiArray),
    "gripper_command"));
  esp_task_wdt_reset();

  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  esp_task_wdt_reset();  // Feed watchdog
  
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  esp_task_wdt_reset();  // Feed watchdog
  
  RCCHECK(rclc_executor_add_subscription(&executor, &step_joint_subscriber, &step_joint_cmd, step_joint_cmd_callback, ON_NEW_DATA));
  esp_task_wdt_reset();  // Feed watchdog
  
  RCCHECK(rclc_executor_add_subscription(&executor, &gripper_subscriber, &gripper_cmd, gripper_cmd_callback, ON_NEW_DATA));
  esp_task_wdt_reset();  // Feed watchdog

  Serial.println("[STARTUP] micro-ROS initialization complete!");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
  vTaskDelay(pdMS_TO_TICKS(1));  // Yield to watchdog, let other tasks run
}

void IRAM_ATTR onStepTimer1() {
  // Direct register access - avoid function calls in ISR
  static boolean state = 1;
  state = !state;
  if (state) {
    GPIO.out_w1ts = (1 << TB6600_PUL1);
  } else {
    GPIO.out_w1tc = (1 << TB6600_PUL1);
  }
}

void IRAM_ATTR onStepTimer2() {
  // Direct register access - avoid function calls in ISR
  static boolean state = 1;
  state = !state;
  if (state) {
    GPIO.out_w1ts = (1 << TB6600_PUL2);
  } else {
    GPIO.out_w1tc = (1 << TB6600_PUL2);
  }
}

void controlTask(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(1000/controlFreq);
  float_t speed1, speed2;
  boolean wasMoving1 = false, wasMoving2 = false;

  for(;;) {
    // Thread-safe read of setSpeed variables
    if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      speed1 = setSpeed1;
      speed2 = setSpeed2;
      xSemaphoreGive(speedMutex);
    } else {
      speed1 = 0.0;
      speed2 = 0.0;
    }
    
    // Thread-safe write to debugData structs
    // I2C timeout is 2ms per sensor, so max ~4ms for both motors total.
    if (xSemaphoreTake(debugDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      motor1->control(speed1, &debugData1);
      motor2->control(speed2, &debugData2);
      
      // Enable/disable timer based on whether motor is moving
      boolean isMoving1 = motor1->getPulseGenerationEnabled();
      boolean isMoving2 = motor2->getPulseGenerationEnabled();
      
      if (isMoving1 && !wasMoving1) {
        timerAlarmEnable(stepTimer1);
      } else if (!isMoving1 && wasMoving1) {
        timerAlarmDisable(stepTimer1);
      }
      
      if (isMoving2 && !wasMoving2) {
        timerAlarmEnable(stepTimer2);
      } else if (!isMoving2 && wasMoving2) {
        timerAlarmDisable(stepTimer2);
      }
      
      wasMoving1 = isMoving1;
      wasMoving2 = isMoving2;
      
      xSemaphoreGive(debugDataMutex);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency); 
  }
}

void error_loop() {
  // Flash LED or send error message if available
  Serial.println("[ERROR] ROS initialization failed. Halting.");
  while(1) {
    delay(100);
  }
}

void step_joint_cmd_callback(const void * msgin) {
  const sensor_msgs__msg__JointState * msg = (const sensor_msgs__msg__JointState *) msgin;
  
  // Validate message has expected data
  if (msg == NULL || msg->velocity.size < 2) {
    return;
  }
  
  // Thread-safe update of setSpeed variables
  if (xSemaphoreTake(speedMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    setSpeed1 = msg->velocity.data[0];
    setSpeed2 = msg->velocity.data[1];
    xSemaphoreGive(speedMutex);
  }
}

void grip(int16_t angle1, int16_t angle2) {
  int16_t maxAngle = 270; 
  angle1 = constrain(angle1, 0, maxAngle);
  angle2 = constrain(angle2, 0, maxAngle);
  int16_t pulseWidth1 = map(angle1, 0, maxAngle, 500, 2500);
  int16_t pulseWidth2 = map(angle2, 0, maxAngle, 500, 2500);
  servo1.writeMicroseconds(pulseWidth1);
  servo2.writeMicroseconds(pulseWidth2);
}

void gripper_cmd_callback(const void * msgin) {
  const std_msgs__msg__UInt16MultiArray * msg = (const std_msgs__msg__UInt16MultiArray *) msgin;
  // Validate message before access
  if (msg == NULL || msg->data.size < 2 || msg->data.data == NULL) {
    return;
  }
  int16_t angle1 = msg->data.data[0];
  int16_t angle2 = msg->data.data[1];
  grip(angle1, angle2);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Thread-safe read of debugData structs
    if (xSemaphoreTake(debugDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      step_joint_feedback.position.data[0] = debugData1.turn;
      step_joint_feedback.position.data[1] = debugData2.turn;
      step_joint_feedback.velocity.data[0] = debugData1.filteredSpeed;
      step_joint_feedback.velocity.data[1] = debugData2.filteredSpeed;
      xSemaphoreGive(debugDataMutex);
    }
    RCSOFTCHECK(rcl_publish(&publisher, &step_joint_feedback, NULL));
  }
}
