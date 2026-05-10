#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "driver/gpio.h"

#define AS5600_ADDR 0x36
#define RAW_ANGLE_MSB_REG 0x0C
#define I2C_READ_TIMEOUT_MS 2        // 2ms timeout: Safe from hangs, fast enough for mutex contention
#define I2C_READ_ERROR_THRESHOLD 10  // Max consecutive errors before fallback
#define SENSOR_ERROR_VALUE 0xFFFF    // Sentinel value indicating read error

struct MotorDebugData {
  float_t targetSpeed = 0.0;
  float_t filteredSpeed = 0.0;
  float_t commandSpeed = 0.0;
  float_t dt = 0.0;
  float_t turn = 0.0; 
};

class MotorController {
private:
  int motorID;
  TwoWire &wire;
  uint8_t pulPin;
  uint8_t dirPin;
  hw_timer_t *timer;
  portMUX_TYPE *mux;

  float_t previousRadAngle = 0.0;
  float_t totalRadAngle = 0.0;
  float_t previousFilteredSpeed = 0.0;
  unsigned long lastTime = 0;
  float_t targetSpeed = 0.0;
  float_t acceleration = 50.0;
  float_t integral = 0.0;
  float_t previousError = 0.0;
  float_t prescale = 1.0;
  float_t speedGap = 5.0;
  float_t cutoffFreq;
  float_t Kp = 1.5;
  float_t Ki = 0.2;
  float_t Kd = 0.0;
  boolean shouldGeneratePulses = false;  // Flag to control pulse generation
  
  // I2C error tracking and fault tolerance
  uint16_t i2cConsecutiveErrors = 0;
  uint16_t lastValidRawAngle = 0;  // Fallback to last good reading
  boolean sensorFaulted = false;   // Indicates persistent I2C failure
  unsigned long lastSuccessfulRead = 0;
  
  float_t filteredSpeedOut = 0.0;
  float_t currentDt = 0.0;
  float_t currentCommandSpeed = 0.0;

  uint16_t readAS5600Angle();
  void updateAngleAndSpeed();
  void updateTargetSpeed(float_t setSpeed);
  void calculatePID();
  void applyCommandSpeed();

public:
  MotorController(int id, TwoWire &w, uint8_t pPin, uint8_t dPin, hw_timer_t *t, portMUX_TYPE *m, float_t cutoff);
  void control(float_t setSpeed, MotorDebugData* debugData);
  boolean getPulseGenerationEnabled() const { return shouldGeneratePulses; }
  boolean getSensorFaultStatus() const { return sensorFaulted; }
  uint16_t getConsecutiveI2CErrors() const { return i2cConsecutiveErrors; }
};

#endif
