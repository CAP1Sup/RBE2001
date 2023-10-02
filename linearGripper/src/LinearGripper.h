#pragma once

#include <Arduino.h>
#include <Servo32U4.h>

// Type definitions
typedef enum { OPEN, CLOSED } GripperState;

/**
 * @brief Class to control the rotary gripper.
 *
 * This class provides methods to control the rotary gripper. It uses a
 * potentiometer to detect the position of the gripper and a servo motor to
 * control the opening and closing of the gripper.
 */
class LinearGripper {
 public:
  LinearGripper(uint8_t feedbackPin, uint16_t openPotVal,
                uint16_t closedPotVal);
  void init();
  void setSpeed(int speed);
  uint16_t getPosition();
  void setDesiredState(GripperState state);

 private:
  Servo32U4Pin5 servo;
  uint8_t feedbackPin;
  uint16_t openPotVal;
  uint16_t closedPotVal;
};