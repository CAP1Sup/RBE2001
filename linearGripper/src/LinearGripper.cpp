/**
 * @file LinearGripper.cpp
 * @brief This file contains the implementation of the LinearGripper class,
 * which controls a servo motor and a potentiometer to open and close a gripper.
 */

#include <LinearGripper.h>

/**
 * @brief Construct a new Linear Gripper:: Linear Gripper object
 *
 * @param feedbackPin The pin that the potentiometer is connected to
 */
LinearGripper::LinearGripper(uint8_t feedbackPin) {
  // Save the feedback pin
  this->feedbackPin = feedbackPin;
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
  if (eStop) {
    return;
  }
  servo.writeMicroseconds(map(speed, -100, 100, 1000, 2000));
}

/**
 * @brief Gets the current position of the gripper
 *
 * @return int16_t The current position of the gripper
 */
int16_t LinearGripper::getPosition() { return analogRead(feedbackPin); }

/**
 * @brief Sets the desired state of the gripper
 *
 * @param state The desired state of the gripper
 * @return true if the gripper successfully reached the desired state
 */
bool LinearGripper::setDesiredState(GripperState state) {
#ifdef DEBUG
  Serial.println(getPosition());
#endif
  if (eStop) {
    return false;
  }
  if (state == OPEN) {
    // Check if the gripper has just started opening
    if (prevSetState != OPEN) {
      // Move the motor to open the gripper
      setSpeed(-SERVO_SPEED);
    }

    // Update the previous state
    prevSetState = state;

    // Check if the gripper is open
    if (OPEN_POT_VAL - getPosition() > -POSITION_TOLERANCE) {
      // Stop the motor if it is
      setSpeed(0);
      currentState = OPEN;
      return true;
    } else {
      return false;
    }
  } else {  // state == CLOSED

    // Check if the gripper has just started closing
    if (prevSetState != CLOSED) {
      // Move the motor to close the gripper
      setSpeed(SERVO_SPEED);

      // Reset the rate calculations/flags
      noInterrupts();
      potRateStartTime = millis() + EXTRA_START_SAMPLING_PERIOD;
      prevPotPos = getPosition();
      closeFailed = false;
      interrupts();
    }

    // Make sure that the close didn't fail
    if (!closeFailed) {
      // Check if the gripper is closed
      if (CLOSED_POT_VAL - getPosition() < POSITION_TOLERANCE) {
        // Stop the motor
        setSpeed(0);
        currentState = CLOSED;
        return true;
      }

      // Check if the gripper is stuck
      if ((int64_t)millis() - potRateStartTime > SPEED_SAMPLING_PERIOD) {
#ifdef DEBUG
        Serial.print("Gripper speed: ");
        Serial.println((getPosition() - prevPotPos));
        potRateStartTime = millis();
        prevPotPos = getPosition();
#else
        // Check if the gripper is stuck
        if (abs(getPosition() - prevPotPos) < SPEED_TOLERANCE) {
          // Gripper is stuck, open the it
          setSpeed(-SERVO_SPEED);
          closeFailed = true;
        } else {
          noInterrupts();
          // Reset the start time
          potRateStartTime = millis();

          // Save the current encoder position
          prevPotPos = getPosition();
          interrupts();
        }
#endif
      }

      // Update the previous state
      prevSetState = state;

      // Return false, the gripper is not in place
      return false;
    } else {  // Gripper is stuck, wait for it open
      if (OPEN_POT_VAL - getPosition() > POSITION_TOLERANCE) {
        // Stop the motor
        setSpeed(0);
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
GripperState LinearGripper::getCurrentState() { return currentState; }

/**
 * @brief Sets the e-stop state of the gripper
 *
 * @param eStop The e-stop state to set
 */
void LinearGripper::setEStop(bool eStop) {
  if (eStop) {
    setSpeed(0);
  }
  this->eStop = eStop;
}