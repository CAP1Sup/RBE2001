#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <PIDcontroller.h>
#include <Servo32U4.h>

// Motor settings
#define DEFAULT_KP 0.5
#define ANGLE_TOLERANCE 1
#define LOWERING_DEADBAND 31
#define RAISING_DEADBAND 45
#define ENCODER_TICKS_PER_REV 540  // ticks per revolution
#define ENCODER_GEAR_RATIO 36      // gear ratio
#define ENCODER_DEG_TO_TICK \
  (ENCODER_TICKS_PER_REV / 360.0f * ENCODER_GEAR_RATIO)
// #define DEBUG

class BlueMotor {
 public:
  // Constructor
  BlueMotor(uint8_t encA, uint8_t encB);

  // Setup the motor
  void init();

  // Set the effort of the motor
  void setEffort(int8_t effort);

  // Set the effort of the motor (with deadband compensation)
  void setEffortDBC(int8_t effort);

  // Calculates the corrected effort after deadband compensation
  int8_t calculateDBCEffort(int8_t userEffort);

  // Move to a given 4 bar angle
  bool moveTo(float desiredAngle);

  // Get the encoder's count
  int32_t getPosition();

  // Set the encoder's count
  void setPosition(int32_t encPos);

  // Get the angle of the 4 bar
  float getAngle();

  // Set the angle of the 4 bar
  void setAngle(float angle);

  // PID constant setters
  void setKp(float k) { pid.setKp(k); }
  void setKi(float k) { pid.setKi(k); }
  void setKd(float k) { pid.setKd(k); }

  // Override functions
  // Called by the remote
  // REGULAR SET EFFORT CALLS WILL NOT WORK WHILE OVERRIDE IS ACTIVE
  // THEY WILL CALL INFINITE LOOPS
  void setOverride();
  void setOverrideEffort(int8_t effort);
  bool isOverridden();
  void clearOverride();

 private:
  // Servo motor object (for MC29)
  Servo32U4Pin6 servo;

  // Encoder object
  Encoder encoder;

  // Current angle of the 4 bar
  float currentAngle;

  // PID controller
  PIDController pid = PIDController(DEFAULT_KP);

  // Override flag
  bool override = false;

  // Last effort set
  // Used when override is released
  int8_t lastEffort = 0;
};

// Include the source file
#include "BlueMotor.cpp"