/**
 * @file RotaryGripper.cpp
 * @brief This file contains the implementation of the RotaryGripper class,
 * which controls a servo motor and a potentiometer to open and close a gripper.
 */

#include <RotaryGripper.h>

#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define TIME_TOLERANCE 1000   // Ms before aborting closing the gripper
// #define DEBUG

/**
 * @brief Construct a new Rotary Gripper:: Rotary Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 * @param lowerPotVal The lower bound of the potentiometer (0 to 1023)
 * @param upperPotVal The upper bound of the potentiometer (0 to 1023)
 * @param closedServoAngle The angle of the servo when the gripper is closed
 * (0-180)
 * @param openServoAngle The angle of the servo when the gripper is open (0-180)
 */
RotaryGripper::RotaryGripper(uint8_t feedbackPin, uint16_t lowerPotVal,
                             uint16_t upperPotVal, uint16_t lockedServoAngle,
                             uint16_t closedServoAngle,
                             uint16_t openServoAngle) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;

  // Save the bounds
  this->closedPotVal = lowerPotVal;
  this->openPotVal = upperPotVal;

  // Save the angles
  this->lockedServoAngle = lockedServoAngle;
  this->closedServoAngle = closedServoAngle;
  this->openServoAngle = openServoAngle;
}

/**
 * @brief Initializes the gripper by attaching the servo motor and performing an
 * analogRead to initialize the ADC.
 *
 */
void RotaryGripper::init() {
  // Attach the servo
  servo.attach();

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
void RotaryGripper::setAngle(int angle) {
  servo.writeMicroseconds(map(angle, 0, 180, 1000, 2000));
}

/**
 * @brief Gets the current angle of the servo motor from the potentiometer
 *
 * @return uint16_t The current angle of the servo motor's potentiometer (0 to
 * 1023)
 */
uint16_t RotaryGripper::getAngle() { return analogRead(feedbackPin); }

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 */
void RotaryGripper::setDesiredState(GripperState state) {
  if (state == OPEN) {
    // Move the motor to open the gripper
    setAngle(openServoAngle);

  } else {
    // Move the motor to lock the gripper
    setAngle(lockedServoAngle);

    // Record the time
    uint32_t startTime = millis();
    bool closeFailed = false;

    // Continuously check if the gripper is in place
    while (abs(closedPotVal - analogRead(feedbackPin)) > POSITION_TOLERANCE) {
      // Check if the gripper is stuck
      if (millis() - startTime > TIME_TOLERANCE) {
        // Open the gripper
        setAngle(openServoAngle);
        closeFailed = true;
        break;
      }
#ifdef DEBUG
      Serial.println(getAngle());
#endif
    }

    // Allow the motor to relax if the gripper is not stuck
    if (!closeFailed) {
      // Back off on the motor to reduce the strain on the servo
      setAngle(closedServoAngle);
    }
  }
}