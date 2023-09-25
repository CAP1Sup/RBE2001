/**
 * @file RotaryGripper.cpp
 * @brief This file contains the implementation of the RotaryGripper class, which controls a servo motor and a potentiometer to open and close a gripper.
 */

#include <RotaryGripper.h>

#define POSITION_TOLERANCE 5  // ADC values (0-1023)

/**
 * @brief Construct a new Rotary Gripper:: Rotary Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 * @param lowerPotVal The lower bound of the potentiometer (0 to 1023)
 * @param upperPotVal The upper bound of the potentiometer (0 to 1023)
 * @param closedServoAngle The angle of the servo when the gripper is closed (0-180)
 * @param openServoAngle The angle of the servo when the gripper is open (0-180)
 */
RotaryGripper::RotaryGripper(uint8_t feedbackPin, uint16_t lowerPotVal,
                             uint16_t upperPotVal, int closedServoAngle,
                             int openServoAngle) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;

  // Save the bounds
  this->closedPotVal = lowerPotVal;
  this->openPotVal = upperPotVal;

  // Save the angles
  this->closedServoAngle = closedServoAngle;
  this->openServoAngle = openServoAngle;
}

/**
 * @brief Initializes the gripper
 *
 * This function initializes the gripper by attaching the servo motor and performing an analogRead to initialize the ADC.
 */
void RotaryGripper::init() {
  // Attach the servo
  servo.attach(5);

  // Perform an analogRead to initialize the ADC
  analogRead(feedbackPin);

  // Open the gripper
  setDesiredState(OPEN);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

/**
 * @brief Sets the servo motor's desired angle
 *
 * @param angle The angle to set the servo to (0 to 180)
 */
void RotaryGripper::setServoAngle(int angle) {
  servo.write(angle);
}

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 */
void RotaryGripper::setDesiredState(GripperState state) {
  if (state == OPEN) {
    // Move the motor to open the gripper
    setServoAngle(openServoAngle);

    // Continuously check if the gripper is in place
    while (abs(openPotVal - analogRead(feedbackPin)) > POSITION_TOLERANCE) {
// Do nothing
#ifdef DEBUG
      Serial.println(getAngle());
#endif
    }
  } else {
    // Move the motor to close the gripper
    setServoAngle(closedServoAngle);

    // Continuously check if the gripper is in place
    while (abs(closedPotVal - analogRead(feedbackPin)) > POSITION_TOLERANCE) {
      // Do nothing
#ifdef DEBUG
      Serial.println(getAngle());
#endif
    }
  }
}