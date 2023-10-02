/**
 * @file LinearGripper.cpp
 * @brief This file contains the implementation of the LinearGripper class,
 * which controls a servo motor and a potentiometer to open and close a gripper.
 */

#include <LinearGripper.h>

#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define SERVO_SPEED 100       // 0-100%
#define SPEED_TOLERANCE 1     // Encoder counts must be over 5 / sampling period
#define SPEED_SAMPLING_PERIOD 100  // ms
// #define DEBUG

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
 *
 * @return true if the gripper successfully reached the desired state
 */
bool LinearGripper::setDesiredState(GripperState state) {
  return setDesiredState(state, false);
}

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 * @param internalCall Whether or not this is an internal call
 *
 * @return true if the gripper successfully reached the desired state
 */
bool LinearGripper::setDesiredState(GripperState state, bool internalCall) {
  if (state == OPEN) {
    // Check if the gripper has just started opening
    if (prevSetState != OPEN) {
      // Move the motor to open the gripper
      setSpeed(-SERVO_SPEED);
    }

    // Check if the gripper is in place
    if (getPosition() - openPotVal < POSITION_TOLERANCE) {
      // Stop the motor if it is
      setSpeed(0);

      // Return true
      return true;
    }
  } else {  // state == CLOSED

    // Check if the gripper has just started closing
    if (prevSetState != CLOSED) {
      // Reset the stuck flag
      isStuck = false;

      // Reset the start time
      potRateStartTime = millis();

      // Reset the previous potentiometer position
      prevPotPos = getPosition();

      // Move the motor to lock the gripper
      setSpeed(SERVO_SPEED);
    }

    // Open the gripper if it is stuck
    /*if (isStuck) {
      // Open the gripper
      return setDesiredState(OPEN, true);
    }*/

    // Check if the gripper is stuck
    /*if (millis() - potRateStartTime > SPEED_SAMPLING_PERIOD) {
      // Check if the gripper is stuck
      if (abs(getPosition() - prevPotPos) < SPEED_TOLERANCE) {
        // Set the stuck flag
        isStuck = true;
      } else {
        // Reset the start time
        potRateStartTime = millis();

        // Save the current encoder rate
        prevPotPos = getPosition();
      }
    }*/

    // Check if the gripper is in place
    if (closedPotVal - getPosition() > POSITION_TOLERANCE) {
      // Stop the motor if it is
      setSpeed(0);

      // Return true
      return true;
    }
  }

  // Don't update the previous set state if this is an internal call
  if (!internalCall) {
    // Save the last set state
    prevSetState = state;
  }

#ifdef DEBUG
  Serial.println(getPosition());
#endif

  // If we've made it this far, the gripper is not in place
  return false;
}