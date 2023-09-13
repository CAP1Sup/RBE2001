#include "BlueMotor.h"

#define ENCODER_POS_TOLERANCE 10

/**
 * @brief Construct a new Blue Motor object
 *
 * @param dirPinA The first direction pin
 * @param dirPinB The second direction pin
 * @param reversed Whether or not the motor is reversed
 */
BlueMotor::BlueMotor(uint8_t dirPinA, uint8_t dirPinB, bool reversed) {
  // Set the pin numbers
  this->dirPinA = dirPinA;
  this->dirPinB = dirPinB;

  // Save if the motor is reversed
  this->reversed = reversed;
}

// Setup the motor
void BlueMotor::init() {
  // Set the pin modes
  pinMode(11, OUTPUT);
  pinMode(dirPinA, OUTPUT);
  pinMode(dirPinB, OUTPUT);

  // Configure the pin 11 timer
  TCCR1A =
      0xA8;  // 0b10101000; //gcl: added OCR1C for adding a third PWM on pin 11
  TCCR1B = 0x11;  // 0b00010001;
  ICR1 = 400;
  OCR1C = 0;

  // Reset the encoder count
  resetPos();
}

// Set the effort of the motor
void BlueMotor::setEffort(int8_t effort) {
  // Set the direction pins
  setDirection(effort < 0);

  // Set the PWM pin
  OCR1C = constrain(abs(effort) * 4, 0, 400);
}

/**
 * @brief Set the direction of the motor
 *
 * @param reverse Whether or not to reverse the motor
 */
void BlueMotor::setDirection(bool reverse) {
  // Set the direction pins
  digitalWrite(dirPinA, reverse ? !reversed : reversed);
  digitalWrite(dirPinB, !reverse ? !reversed : reversed);
}

/**
 * @brief Move to a given encoder count
 *
 * @param position The encoder count to move to
 * @param effort Maximum effort to use (0-100)
 */
void BlueMotor::moveTo(int32_t position, uint8_t effort) {
  // Set the motor effort
  setEffort(effort * (position > getPosition() ? 1 : -1));

  // Wait until the desired encoder count is reached
  while (abs(getPosition() - position) > ENCODER_POS_TOLERANCE) {
    // Do nothing
  }

  // Stop the motor
  setEffort(0);
}

/**
 * @brief Get the encoder's count
 *
 * @return int32_t The encoder's count
 */
int32_t BlueMotor::getPosition() {
  // Return the encoder count
  return encoder.read();
}

/**
 * @brief Reset the encoder's position
 *
 */
void BlueMotor::resetPos() {
  // Reset the encoder count
  encoder.write(0);
}
