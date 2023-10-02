#pragma once

#include <Arduino.h>
#include <Servo32U4.h>

// Compiler definitions
#define OPEN_POT_VAL 600
#define CLOSED_POT_VAL 800
#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define SERVO_SPEED 100       // 0-100%
#define SPEED_TOLERANCE 1     // Encoder counts must be over 5 / sampling period
#define SPEED_SAMPLING_PERIOD 100  // ms
// #define DEBUG

// Type definitions
typedef enum { OPEN, CLOSED, UNKNOWN } GripperState;

/**
 * @brief Class to control the rotary gripper.
 *
 * This class provides methods to control the rotary gripper. It uses a
 * potentiometer to detect the position of the gripper and a servo motor to
 * control the opening and closing of the gripper.
 */
class LinearGripper {
 public:
  LinearGripper(uint8_t feedbackPin);
  void init();
  void setSpeed(int speed);
  uint16_t getPosition();
  bool setDesiredState(GripperState state);

 private:
  bool setDesiredState(GripperState state, bool internalCall);
  Servo32U4Pin5 servo;
  uint8_t feedbackPin;
  bool isStuck = false;
  GripperState prevSetState = UNKNOWN;
  uint32_t potRateStartTime;
  uint16_t prevPotPos;
};

// Include the source file
#include "LinearGripper.cpp"