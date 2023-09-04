#pragma once

#include <Arduino.h>
#include <Servo.h>

// Type definitions
typedef enum { OPEN, CLOSED } GripperState;

/**
 * @brief Class to control the rotary gripper.
 *
 */
class RotaryGripper {
 public:
  RotaryGripper(uint8_t feedbackPin, uint16_t lowerPotVal, uint16_t upperPotVal,
                float lowerAngle, float upperAngle);
  void init();
  void setMotorPower(int power);
  void setDesiredState(GripperState state);
  float getAngle();

 private:
  Servo servo;
  uint8_t feedbackPin;
  uint16_t potLowerBound;
  uint16_t potUpperBound;
  float angleLowerBound;
  float angleUpperBound;
};

// Helper functions
float mapf(float x, float in_min, float in_max, float out_min, float out_max);