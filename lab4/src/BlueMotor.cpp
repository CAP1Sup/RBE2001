#include "BlueMotor.h"

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
  // Constrain the effort
  effort = constrain(effort, -100, 100);

  // Set the servo pulse
  servo.writeMicroseconds(1500 + effort * 5);
}

// Set the effort of the motor controller (with deadband compensation)
void BlueMotor::setEffortDBC(int8_t effort) {
  // Constrain the effort
  effort = constrain(effort, -100, 100);

  effort = calculateDBCEffort(effort);

  // Set the motor's effort
  setEffort(effort);
}

int8_t BlueMotor::calculateDBCEffort(int8_t userEffort) {
  // Check which direction the motor is moving
  if (userEffort >= 0) {
    return
        ((100 - RAISING_DEADBAND) * (int16_t)(userEffort)) / 100 + RAISING_DEADBAND;
  } else {
    return ((100 - LOWERING_DEADBAND) * (int16_t)(userEffort)) / 100 -
             LOWERING_DEADBAND;
  }
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
