#pragma once

#include <Arduino.h>
#include <Encoder.h>
#include <PIDcontroller.h>
#include <Servo32U4.h>

// Motor settings
#define DEFAULT_KP 0.5
#define ENCODER_POS_TOLERANCE 10
#define LOWERING_DEADBAND 10
#define RAISING_DEADBAND 10

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

  // Move to a given encoder count
  void moveTo(int32_t position);

  // Get the encoder's count
  int32_t getPosition();

  // Reset the encoder state
  void resetPos();

  // PID constant setters
  void setKp(float k) { pid.setKp(k); }
  void setKi(float k) { pid.setKi(k); }
  void setKd(float k) { pid.setKd(k); }

 private:
  // Servo motor object (for MC29)
  Servo32U4Pin6 servo;

  // Encoder object
  Encoder encoder;

  // PID controller
  PIDController pid = PIDController(DEFAULT_KP);
};