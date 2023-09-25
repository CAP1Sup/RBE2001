#pragma once

#include <Arduino.h>
#include <Servo.h>

// Type definitions
typedef enum { OPEN, CLOSED } GripperState;

/**
 * @brief Class to control the rotary gripper.
 *
 * This class provides methods to control the rotary gripper. It uses a potentiometer to detect the position of the gripper and a servo motor to control the opening and closing of the gripper.
 */
class RotaryGripper {
 public:
  RotaryGripper(uint8_t feedbackPin, uint16_t closedPotVal, uint16_t openPotVal,
                int closedServoAngle, int openServoAngle);
  void init();
  void setServoAngle(int angle);
  void setDesiredState(GripperState state);

 private:
  Servo servo;
  uint8_t feedbackPin;
  uint16_t closedPotVal;
  uint16_t openPotVal;
  int closedServoAngle;
  int openServoAngle;
};