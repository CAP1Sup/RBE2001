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

  // Stop the motor
  setEffort(0);

  // Set the default PID constants
  pid.setKp(DEFAULT_KP);
  pid.setKi(0.0);
  pid.setKd(0.0);

  // Reset the encoder count
  setPosition(0);
}

// Set the effort of the motor controller
void BlueMotor::setEffort(int8_t effort) {
  // Constrain the effort
  effort = constrain(effort, -99, 99);

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
    return ((100 - RAISING_DEADBAND) * (int16_t)(userEffort)) / 100 +
           RAISING_DEADBAND;
  } else {
    return ((100 - LOWERING_DEADBAND) * (int16_t)(userEffort)) / 100 -
           LOWERING_DEADBAND;
  }
}

/**
 * @brief Move the 4 bar to a given angle
 *
 * @param desiredAngle The angle to move to
 */
void BlueMotor::moveTo(float desiredAngle) {
  // Wait until the desired encoder count is reached
  float currentAngle;
  while (true) {
    // Get the current angle
    currentAngle = getAngle();

    // Keep setting the motor effort
    float effort = pid.calcEffort(desiredAngle - currentAngle);

    // Constrain the effort
    setEffort(constrain(effort, -100, 100));

    // Check if it's time to exit the loop
    if (abs(desiredAngle - currentAngle) < ANGLE_TOLERANCE) {
      break;
    }

// Print out helpful info
#ifdef DEBUG
    Serial.print("Desired: ");
    Serial.print(desiredAngle);
    Serial.print(", Current: ");
    Serial.print(currentAngle);
    Serial.print(", Effort: ");
    Serial.println(effort);
#endif

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
 * @param encPos The position to set the encoder to
 */
void BlueMotor::setPosition(int32_t encPos) {
  // Set the encoder's count
  encoder.setPosition(encPos);
}

// Set the encoder conversion factor
void BlueMotor::setDegToEncCount(float degToEncCount) {
  // Set the conversion factor
  this->degToEncCount = degToEncCount;
}

// Get the angle of the 4 bar (in degrees)
float BlueMotor::getAngle() {
  // Return the angle
  return getPosition() / degToEncCount;
}

// Set the angle of the 4 bar
void BlueMotor::setAngle(float angle) {
  // Set the angle
  setPosition(angle * degToEncCount);
}
