#pragma once

#include <Arduino.h>
#include <Encoder.h>

class BlueMotor {
 public:
  // Constructor
  BlueMotor(uint8_t dirPinA, uint8_t dirPinB, uint8_t encA, uint8_t encB);

  // Setup the motor
  void init();

  // Set the effort of the motor
  void setEffort(int8_t effort);

  // Move to a given encoder count
  void moveTo(int32_t position, uint8_t effort);

  // Get the encoder's count
  int32_t getPosition();

  // Reset the encoder state
  void resetPos();

 private:
  // Set the direction of the motor
  void setDirection(bool reverse);

  // Pin numbers
  uint8_t dirPinA;
  uint8_t dirPinB;

  // Encoder object
  Encoder encoder;
};