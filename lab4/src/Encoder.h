#pragma once

#include "Arduino.h"

class Encoder {
 public:
  Encoder();
  static void init(uint8_t pinA, uint8_t pinB);
  static int32_t getPosition();
  static void resetPosition();
  static void isrA();
  static void isrB();

 private:
  static uint8_t _pinA;
  static uint8_t _pinB;
  static int32_t position;
};
