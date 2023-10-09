#include "moveFunctions.h"

#include <Arduino.h>

// Globals
int lVal, rVal;

/**
 * @brief Follows the line until the cross is found
 *
 * @param searchDir The direction to search for the line
 * @return int The encoder count when the cross is found
 */
int followUntilCross(TURN_DIR searchDir) {
  // White: ~40
  // Black: ~800

  // We're not on the line yet
  bool onLine = false;

  // Loop until we find the cross
  while (true) {
    // Get the values from the reflectance sensors
    // analogRead() calls are expensive
    lVal = analogRead(L_LINE_FOLLOW_PIN);
    rVal = analogRead(R_LINE_FOLLOW_PIN);

    if (onLine) {
      // We're on the line
      if (lVal > BLACK_THRESHOLD && rVal > BLACK_THRESHOLD) {
        // We're at the cross
        chassis.idle();

        // Break the loop and return the encoder count
        // Add a constant to account for the distance between the line sensor
        // and the center of the robot
        return chassis.getLeftEncoderCount(true) + 250;

      } else {
        // Calculate the difference between the two line sensors
        int difference =
            analogRead(L_LINE_FOLLOW_PIN) - analogRead(R_LINE_FOLLOW_PIN);
        chassis.setTwist(FORWARD_SPEED * INCHES_TO_CM,
                         LINE_FOLLOW_P * difference);
      }
    } else {
      // We're off the line, turn until we find it
      chassis.setMotorEfforts(searchDir * -SEARCH_EFFORT,
                              searchDir * SEARCH_EFFORT);

      // Check if we're on the line
      if (lVal > BLACK_THRESHOLD || rVal > BLACK_THRESHOLD) {
        onLine = true;
        // Reset the left encoder's value
        chassis.getLeftEncoderCount(true);
      }
    }

    // Limit the update rate to allow motors time to move
    delay(10);
  }
}

/**
 * @brief Follows the line until the given encoder count is reached
 *
 * @param searchDir The direction to search for the line
 * @param encoderCount The encoder count to stop at
 */
void followUntilCount(TURN_DIR searchDir, int encoderCount) {
  // White: ~40
  // Black: ~800

  // We're not on the line yet
  bool onLine = false;

  // Loop until we find the cross
  while (true) {
    // Get the values from the reflectance sensors
    // analogRead() calls are expensive
    lVal = analogRead(L_LINE_FOLLOW_PIN);
    rVal = analogRead(R_LINE_FOLLOW_PIN);

    if (onLine) {
      // We're on the line
      if (chassis.getLeftEncoderCount() >= encoderCount) {
        // We went the distance we needed to
        chassis.idle();

        // Break the loop
        return;
      } else {
        // Calculate the difference between the two line sensors
        int difference =
            analogRead(L_LINE_FOLLOW_PIN) - analogRead(R_LINE_FOLLOW_PIN);
        chassis.setTwist(FORWARD_SPEED * INCHES_TO_CM,
                         LINE_FOLLOW_P * difference);
      }
    } else {
      // We're off the line, turn until we find it
      chassis.setMotorEfforts(searchDir * -SEARCH_EFFORT,
                              searchDir * SEARCH_EFFORT);

      // Check if we're on the line
      if (lVal > BLACK_THRESHOLD || rVal > BLACK_THRESHOLD) {
        onLine = true;
        // Reset the left encoder's value
        chassis.getLeftEncoderCount(true);
      }
    }

    // Limit the update rate to allow motors to move
    delay(10);
  }
}

/**
 * @brief Follows the line until the given minimum distance on the ultrasonic
 * sensor is reached
 *
 * @param searchDir The direction to search for the line
 * @param distance The minimum distance to stop at (in inches)
 */
void followUntilDist(TURN_DIR searchDir, float distance) {
  // White: ~40
  // Black: ~800

  // We're not on the line yet
  bool onLine = false;

  // Loop until we find the cross
  while (true) {
    // Get the values from the reflectance sensors
    // analogRead() calls are expensive
    lVal = analogRead(L_LINE_FOLLOW_PIN);
    rVal = analogRead(R_LINE_FOLLOW_PIN);

    if (onLine) {
      // We're on the line
      if (rangefinder.getDistance() <= distance * INCHES_TO_CM) {
        // We went the distance we needed to
        chassis.idle();

        // Break the loop
        return;
      } else {
        // Calculate the difference between the two line sensors
        int difference =
            analogRead(L_LINE_FOLLOW_PIN) - analogRead(R_LINE_FOLLOW_PIN);
        chassis.setTwist(FORWARD_SPEED * INCHES_TO_CM,
                         LINE_FOLLOW_P * difference);
      }
    } else {
      // We're off the line, turn until we find it
      chassis.setMotorEfforts(searchDir * -SEARCH_EFFORT,
                              searchDir * SEARCH_EFFORT);

      // Check if we're on the line
      if (lVal > BLACK_THRESHOLD || rVal > BLACK_THRESHOLD) {
        onLine = true;
        // Reset the left encoder's value
        chassis.getLeftEncoderCount(true);
      }
    }

    // Limit the update rate to allow motors to move
    delay(10);
  }
}

/**
 * @brief Moves the robot forward over the cross and turns to the given
 *
 * @param turnDir The direction to turn
 */
void turnOnCross(TURN_DIR turnDir) {
  // Move forward over the cross
  chassis.driveFor(3 * INCHES_TO_CM, FORWARD_SPEED * INCHES_TO_CM, true);

  // Turn to the given direction
  chassis.turnFor(turnDir * 70, TURN_SPEED, true);
}

/**
 * @brief Drives forward until the cross is found
 *
 * @return int The encoder count when the cross is found
 */
int driveUntilCross() {
  // White: ~40
  // Black: ~800

  // Reset the encoder count
  chassis.getLeftEncoderCount(true);

  // Start moving
  chassis.setWheelSpeeds(FORWARD_SPEED * INCHES_TO_CM,
                         FORWARD_SPEED * INCHES_TO_CM);

  // Loop until we find the cross
  while (true) {
    // Get the values from the reflectance sensors
    // analogRead() calls are expensive
    lVal = analogRead(L_LINE_FOLLOW_PIN);
    rVal = analogRead(R_LINE_FOLLOW_PIN);

    // Check if we're on the cross
    if (lVal > BLACK_THRESHOLD && rVal > BLACK_THRESHOLD) {
      // We're at the cross
      chassis.idle();

      // Break the loop and return the encoder count
      // Add a constant to account for the distance between the line sensor
      // and the center of the robot
      return chassis.getLeftEncoderCount(true) + 250;
    }
  }

  // Limit the update rate to allow motors time to move
  delay(10);
}

void driveUntilDist(float distance) {
  // Reset the encoder count
  chassis.getLeftEncoderCount(true);

  // Start moving
  chassis.setWheelSpeeds(FORWARD_SPEED * INCHES_TO_CM,
                         FORWARD_SPEED * INCHES_TO_CM);

  // Loop until we find the cross
  while (true) {
    // We're on the line
    if (rangefinder.getDistance() <= distance * INCHES_TO_CM) {
      // We're at the cross
      chassis.idle();

      // Break the loop
      return;
    }

    // Limit the update rate to allow US to reset
    delay(10);
  }
}