#include "RotaryGripper.h"

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
  if (eStop) {
    return;
  }
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
  Serial.print(getAngle());
  Serial.print(" ");
  Serial.print(state);
  Serial.print(" ");
  Serial.print(prevSetState);
  Serial.print(" ");
  Serial.print(lastSetTime);
  Serial.print(" ");
  Serial.println(millis() - lastSetTime);
  delay(10);
#endif
  if (eStop) {
    return false;
  }
  if (state == OPEN) {
    if (prevSetState != OPEN) {
      // Move the motor to open the gripper
      setAngle(OPEN_SERVO_ANGLE);

      noInterrupts();
      // Record the time
      lastSetTime = millis();
      interrupts();
    }

    // Update the previous state
    prevSetState = state;

    // Return if the move has finished
    if (millis() - lastSetTime > OPENING_TIME) {
      currentState = OPEN;
      return true;
    } else {
      return false;
    }

  } else {
    if (prevSetState != CLOSED) {
      // Move the motor to close the gripper
      setAngle(CLOSED_SERVO_ANGLE);

      // Record the time
      noInterrupts();
      lastSetTime = millis();
      closeFailed = false;
      interrupts();
    }

    // Make sure that the close didn't fail
    if (!closeFailed) {
      // Check if the gripper is closed
      if (CLOSED_POT_VAL - analogRead(feedbackPin) > -POSITION_TOLERANCE) {
        // Gripper finished moving
        setAngle(LOCKED_SERVO_ANGLE);
        currentState = CLOSED;
        return true;
      }

      // Check if the gripper is stuck
      if (millis() - lastSetTime > CLOSING_TIME_TOLERANCE) {
        // Gripper is stuck, open the it
        setAngle(OPEN_SERVO_ANGLE);
        lastSetTime = millis();
        closeFailed = true;
      }

      // Update the previous state
      prevSetState = state;

      // Return false, the gripper is not in place
      return false;
    } else {  // Gripper is stuck, wait for it open
      if (millis() - lastSetTime > OPENING_TIME) {
        currentState = OPEN;
        return true;
      } else {
        return false;
      }
    }
  }
}

/**
 * @brief Gets the current state of the gripper
 * Only valid after setDesiredState() returns true
 *
 * @return GripperState The current state of the gripper
 */
GripperState RotaryGripper::getCurrentState() { return currentState; }

/**
 * @brief Sets the e-stop state of the gripper
 *
 * @param eStop The e-stop state to set
 */
void RotaryGripper::setEStop(bool eStop) { this->eStop = eStop; }