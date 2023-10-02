/**
 * @file LinearGripper.cpp
 * @brief This file contains the implementation of the LinearGripper class,
 * which controls a servo motor and a potentiometer to open and close a gripper.
 */

#include <LinearGripper.h>

#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define SERVO_SPEED 50        // 0-100%
#define TIME_TOLERANCE 30000  // Ms before aborting closing the gripper
#define DEBUG

/**
 * @brief Construct a new Linear Gripper:: Linear Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 * @param openPotVal The potentiometer reading when the gripper is open
 * @param closedPotVal The potentiometer reading when the gripper is closed
 */
LinearGripper::LinearGripper(uint8_t feedbackPin, uint16_t openPotVal,
                             uint16_t closedPotVal) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;

  // Save the bounds
  this->openPotVal = openPotVal;
  this->closedPotVal = closedPotVal;
}

/**
 * @brief Initializes the gripper by attaching the servo motor and performing an
 * analogRead to initialize the ADC.
 *
 */
void LinearGripper::init() {
  // Attach the servo
  servo.attach();

  // Perform an analogRead to initialize the ADC
  analogRead(feedbackPin);

  // Set the speed of the motor to 0
  setSpeed(0);

#ifdef DEBUG
  Serial.begin(9600);
#endif
}

/**
 * @brief Sets the servo motor's desired speed
 *
 * @param speed The speed to set the servo to (-100 to 100)
 */
void LinearGripper::setSpeed(int speed) {
  servo.writeMicroseconds(map(speed, -100, 100, 1000, 2000));
}

/**
 * @brief Gets the current position of the gripper
 *
 * @return uint16_t The current position of the gripper
 */
uint16_t LinearGripper::getPosition() { return analogRead(feedbackPin); }

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 */
void LinearGripper::setDesiredState(GripperState state) {
  if (state == OPEN) {
    // Move the motor to open the gripper
    setSpeed(-SERVO_SPEED);

    // Continuously check if the gripper is in place
    while (getPosition() - openPotVal > POSITION_TOLERANCE) {
      // Do nothing
#ifdef DEBUG
      Serial.println(getPosition());
#endif
    }

    // Stop the motor
    setSpeed(0);
  } else {
    // Move the motor to lock the gripper
    setSpeed(SERVO_SPEED);

    // Record the time
    uint32_t startTime = millis();

    // Continuously check if the gripper is in place
    while (closedPotVal - getPosition() > POSITION_TOLERANCE) {
      // Check if the gripper is stuck
      if (millis() - startTime > TIME_TOLERANCE) {
        // Open the gripper
        setDesiredState(OPEN);
        break;
      }
#ifdef DEBUG
      Serial.println(getPosition());
#endif
    }

    // Stop the motor
    setSpeed(0);
  }
}