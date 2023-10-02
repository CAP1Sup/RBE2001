#pragma once

#include <Arduino.h>
#include <Servo32U4.h>

// Compiler definitions
#define CLOSED_POT_VAL 223     // ADC values (0-1023)
#define OPEN_POT_VAL 355       // ADC values (0-1023)
#define CLOSED_SERVO_ANGLE 10  // deg (0-180)
#define LOCKED_SERVO_ANGLE 16  // deg (0-180)
#define OPEN_SERVO_ANGLE 140   // deg (0-180)
#define OPENING_TIME 500       // ms

// Type definitions
typedef enum { OPEN, CLOSED, UNKNOWN } GripperState;

/**
 * @brief Class to control the rotary gripper.
 *
 * This class provides methods to control the rotary gripper. It uses a
 * potentiometer to detect the position of the gripper and a servo motor to
 * control the opening and closing of the gripper.
 */
class RotaryGripper {
 public:
  RotaryGripper(uint8_t feedbackPin);
  void init();
  void setAngle(int angle);
  uint16_t getAngle();
  bool setDesiredState(GripperState state);

 private:
  Servo32U4Pin5 servo;
  uint8_t feedbackPin;
  GripperState prevSetState = UNKNOWN;
  uint32_t lastSetTime;
  bool closeFailed = false;
};

// Include the source file
#include "RotaryGripper.cpp"