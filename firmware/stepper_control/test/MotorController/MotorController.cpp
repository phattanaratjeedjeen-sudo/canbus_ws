#include "MotorController.h"

MotorController::MotorController(int id, TwoWire &w, uint8_t pPin, uint8_t dPin, hw_timer_t *t, portMUX_TYPE *m, float_t cutoff)
  : motorID(id), wire(w), pulPin(pPin), dirPin(dPin), timer(t), mux(m), cutoffFreq(cutoff) {
  lastTime = micros();
}

uint16_t MotorController::readAS5600Angle() {
  uint16_t rawAngle = SENSOR_ERROR_VALUE;  // Initialize with error sentinel
  
  // Use timeout to prevent indefinite blocking on I2C (2ms is safe - sensor responds in microseconds if alive)
  unsigned long timeout = millis() + I2C_READ_TIMEOUT_MS;
  
  wire.beginTransmission(AS5600_ADDR);
  wire.write(RAW_ANGLE_MSB_REG);
  if (wire.endTransmission(false) != 0) {
    // Address write failed
    return SENSOR_ERROR_VALUE;
  }
  
  // Request with timeout protection
  wire.requestFrom(AS5600_ADDR, 2);  // Request 2 bytes from sensor
  
  // Wait for data with timeout (typically completes in microseconds)
  int attempts = 0;
  while (wire.available() < 2 && millis() < timeout && attempts < 50) {
    delayMicroseconds(100);
    attempts++;
  }
  
  if (wire.available() == 2) {
    uint8_t msb = wire.read();
    uint8_t lsb = wire.read();
    rawAngle = (msb << 8) | lsb;
    // Valid read - reset error tracking
    i2cConsecutiveErrors = 0;
    lastValidRawAngle = rawAngle;
    lastSuccessfulRead = millis();
    sensorFaulted = false;
  } else {
    // Read timeout or insufficient data
    rawAngle = SENSOR_ERROR_VALUE;
  }
  
  return rawAngle;
}

void MotorController::updateAngleAndSpeed() {
  uint16_t rawAngle = readAS5600Angle();
  
  // Handle sensor read error with fallback logic
  if (rawAngle == SENSOR_ERROR_VALUE) {
    i2cConsecutiveErrors++;
    
    if (i2cConsecutiveErrors >= I2C_READ_ERROR_THRESHOLD) {
      // Too many consecutive errors - mark sensor as faulted
      sensorFaulted = true;
      Serial.printf("[WARN] Motor %d: I2C sensor communication lost. Using last valid reading.\n", motorID);
    }
    
    // Use last valid reading instead of failing
    rawAngle = lastValidRawAngle;
  }
  
  float_t radAngle = (rawAngle / 4096.0) * 2.0 * PI;
  
  float_t deltaAngle = remainder(radAngle - previousRadAngle, 2.0 * PI);
  totalRadAngle += deltaAngle;
  previousRadAngle = radAngle;

  unsigned long now = micros();
  currentDt = (now - lastTime) / 1000000.0;
  if (currentDt <= 0.0) currentDt = 0.0001;
  lastTime = now;

  float_t tau = 1.0 / (2.0 * PI * cutoffFreq);
  float_t alpha = currentDt / (tau + currentDt);
  float_t speed = deltaAngle / currentDt;
  
  filteredSpeedOut = previousFilteredSpeed + alpha * (speed - previousFilteredSpeed);
  previousFilteredSpeed = filteredSpeedOut;
}

void MotorController::updateTargetSpeed(float_t setSpeed) {
  float_t deltaSpeed = acceleration * currentDt;
  float_t diffSpeed = setSpeed - targetSpeed;
  float_t changedSpeed = fmaxf(-deltaSpeed, fminf(diffSpeed, deltaSpeed));
  targetSpeed += changedSpeed;
}

void MotorController::calculatePID() {
  float_t error = targetSpeed - filteredSpeedOut;
  
  if (fabsf(error) <= speedGap) {
    integral += (error * currentDt);
    integral = fmaxf(fminf(integral, 100.0), -100.0);
    prescale = 1.0;
  } else {
    integral = 0.0;
    prescale = fabsf(1.2 * filteredSpeedOut / (targetSpeed != 0 ? targetSpeed : 0.0001));
  }
  
  float_t derivative = (error - previousError) / currentDt;
  previousError = error;
  float_t outputSpeed = (Kp * error) + (Ki * integral) + (Kd * derivative);
  currentCommandSpeed = fmaxf(fminf(targetSpeed + outputSpeed, 100.0), -100.0) * prescale;
}

void MotorController::applyCommandSpeed() {
  if (fabsf(currentCommandSpeed) <= 0.1) {
    // Disable pulse generation but keep timer running to avoid glitching
    shouldGeneratePulses = false;
    gpio_set_level((gpio_num_t)pulPin, 0);  // Ensure pulse pin is LOW
    return;
  }

  // Enable pulse generation
  shouldGeneratePulses = true;
  
  uint32_t stepFreq = (uint32_t)(fabsf(currentCommandSpeed) * 1600.0 / (2 * PI));
  if (stepFreq == 0) stepFreq = 1;
  gpio_set_level((gpio_num_t)dirPin, currentCommandSpeed > 0 ? 0 : 1);
  
  uint32_t usInterval = 1000000 / (stepFreq * 2);
  portENTER_CRITICAL(mux);
  timerAlarmWrite(timer, usInterval, true);
  portEXIT_CRITICAL(mux);
}

void MotorController::control(float_t setSpeed, MotorDebugData* debugData) {
  updateAngleAndSpeed();
  updateTargetSpeed(setSpeed);
  calculatePID();
  applyCommandSpeed();
  
  if (debugData != nullptr) {
    debugData->targetSpeed = targetSpeed;
    debugData->filteredSpeed = filteredSpeedOut;
    debugData->commandSpeed = currentCommandSpeed;
    debugData->dt = currentDt;
    debugData->turn = totalRadAngle / (2.0 * PI);
  }
}
