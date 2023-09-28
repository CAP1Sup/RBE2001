#include "BlueMotor.h"

#define ENCODER_POS_TOLERANCE 10

/**
 * @brief Construct a new Blue Motor object
 *
 * @param encA The first encoder pin
 * @param encB The second encoder pin
 */
BlueMotor::BlueMotor(uint8_t encA, uint8_t encB) {
  // Initialize the encoder
  encoder.init(encA, encB);
}

// Setup the motor
void BlueMotor::init() {
  // Initialize the MC29's servo object
  servo.attach();

  // Reset the encoder count
  resetPos();
}

// Set the effort of the motor controller
void BlueMotor::setEffort(int8_t effort) {
  // Set the servo pulse
  servo.writeMicroseconds(1500 + effort * 5);
}

/**
 * @brief Move to a given encoder count
 *
 * @param position The encoder count to move to
 * @param effort Maximum effort to use (0-100)
 */
void BlueMotor::moveTo(int32_t position) {
  // Wait until the desired encoder count is reached
  while (abs(position - getPosition()) > ENCODER_POS_TOLERANCE) {
    // Keep setting the motor effort
    setEffort(pid.calcEffort(position - getPosition()));

    // Give a minor delay for the motor to move
    delay(10);
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
  return encoder.getPosition();
}

/**
 * @brief Reset the encoder's position
 *
 */
void BlueMotor::resetPos() {
  // Reset the encoder count
  encoder.resetPosition();
}
