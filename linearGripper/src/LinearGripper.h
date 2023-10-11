#pragma once

#include <Arduino.h>
#include <Servo32U4.h>

// Compiler definitions
#define OPEN_POT_VAL 600
#define CLOSED_POT_VAL 775
#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define SERVO_SPEED 100       // 0-100%
#define SPEED_TOLERANCE \
  5  // Encoder counts must be over this per sampling period
#define SPEED_SAMPLING_PERIOD 500        // ms
#define EXTRA_START_SAMPLING_PERIOD 500  // ms to add to first sampling period
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
  int16_t getPosition();
  bool setDesiredState(GripperState state);
  GripperState getCurrentState();
  void setEStop(bool eStop);

 private:
  Servo32U4Pin5 servo;
  uint8_t feedbackPin;
  GripperState prevSetState = UNKNOWN;
  GripperState currentState = UNKNOWN;
  uint32_t potRateStartTime;
  uint16_t prevPotPos;
  bool closeFailed;
  bool eStop = false;
};

// Include the source file
#include "LinearGripper.cpp"