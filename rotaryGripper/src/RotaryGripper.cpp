#include <RotaryGripper.h>

#define POSITION_TOLERANCE 0.5  // deg
#define MAX_POWER 50            // %

/**
 * @brief Construct a new Rotary Gripper:: Rotary Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 * @param lowerPotVal The lower bound of the potentiometer (0 to 1023)
 * @param upperPotVal The upper bound of the potentiometer (0 to 1023)
 * @param lowerAngle The lower bound of the angle (deg)
 * @param upperAngle The upper bound of the angle (deg)
 */
RotaryGripper::RotaryGripper(uint8_t feedbackPin, uint16_t lowerPotVal,
                             uint16_t upperPotVal, float lowerAngle,
                             float upperAngle) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;

  // Save the bounds
  this->potLowerBound = lowerPotVal;
  this->potUpperBound = upperPotVal;

  // Save the angles
  this->angleLowerBound = lowerAngle;
  this->angleUpperBound = upperAngle;
}

/**
 * @brief Initializes the gripper
 *
 */
void RotaryGripper::init() {
  // Attach the servo
  servo.attach(5);

  // Set the motor power to 0
  setMotorPower(0);

  // Perform an analogRead to initialize the ADC
  analogRead(feedbackPin);
}

/**
 * @brief Sets the servo motor power
 *
 * @param power The power to set the motor to (-100 to 100)
 */
void RotaryGripper::setMotorPower(int power) {
  // Set the motor power
  servo.writeMicroseconds(1500 + power * 500);
}

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 */
void RotaryGripper::setDesiredState(GripperState state) {
  if (state == OPEN) {
    // Move the motor to open the gripper
    setMotorPower(MAX_POWER);

    // Continuously check if the gripper is in place
    while (abs(angleUpperBound - getAngle()) > POSITION_TOLERANCE) {
      // Do nothing
    }

    // Stop the motor
    setMotorPower(0);
  } else {
    // Move the motor to close the gripper
    setMotorPower(-MAX_POWER);

    // Continuously check if the gripper is in place
    while (abs(angleLowerBound - getAngle()) > POSITION_TOLERANCE) {
      // Do nothing
    }

    // Delay a little to allow motor to go past center
    delay(50);

    // Stop the motor
    setMotorPower(0);
  }
}

/**
 * @brief Gets the current angle of the gripper from the potentiometer
 *
 * @return The current angle of the gripper
 */
float RotaryGripper::getAngle() {
  // Convert the potentiometer value to an angle
  return mapf(analogRead(feedbackPin), potLowerBound, potUpperBound,
              angleLowerBound, angleUpperBound);
}

float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}