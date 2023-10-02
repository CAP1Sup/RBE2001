#include "RotaryGripper.h"

#define POSITION_TOLERANCE 5  // ADC values (0-1023)
#define TIME_TOLERANCE 1000   // Ms before aborting closing the gripper
// #define DEBUG

/**
 * @brief Construct a new Rotary Gripper:: Rotary Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 */
RotaryGripper::RotaryGripper(uint8_t feedbackPin) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;
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
 *
 * @return true If the gripper was successfully set to the desired state
 */
bool RotaryGripper::setDesiredState(GripperState state) {
#ifdef DEBUG
  Serial.println(getAngle());
#endif
  if (state == OPEN) {
    if (prevSetState != OPEN) {
      // Move the motor to open the gripper
      setAngle(OPEN_SERVO_ANGLE);

      // Record the time
      lastSetTime = millis();
    }

    // Update the previous state
    prevSetState = state;

    // Return if the move has finished
    return (millis() - lastSetTime > OPENING_TIME);

  } else {
    if (prevSetState != CLOSED) {
      // Move the motor to close the gripper
      setAngle(CLOSED_SERVO_ANGLE);

      // Record the time
      lastSetTime = millis();
      closeFailed = false;
    }

    // Check if the gripper is in place
    if (CLOSED_POT_VAL - analogRead(feedbackPin) > -POSITION_TOLERANCE) {
      // Gripper finished moving
      setAngle(LOCKED_SERVO_ANGLE);
      return true;
    }

    // Check if the gripper is stuck
    if (millis() - lastSetTime > TIME_TOLERANCE) {
      // Open the gripper
      setAngle(OPEN_SERVO_ANGLE);
      closeFailed = true;
    }

    // Update the previous state
    prevSetState = state;

    // Return false, the gripper is not in place
    return false;
  }
}