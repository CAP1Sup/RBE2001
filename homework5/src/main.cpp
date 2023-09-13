#include <Arduino.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Romi32U4.h>

// Settings
#define FORWARD_SPEED 12   // in/s
#define TURN_SPEED 90      // deg/s
#define SEARCH_EFFORT 100  // Motor power, 0-300ish
#define INCHES_TO_CM 2.54
#define LINE_FOLLOW_P 0.1  // deg/s per difference in sensor values
#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3
#define ULTRASONIC_ECHO 3
#define ULTRASONIC_TRIG 2
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800

// Globals
Chassis chassis;
Rangefinder rangefinder(ULTRASONIC_ECHO, ULTRASONIC_TRIG);
int lVal, rVal;

// Type definitions
// Direction inverts the sign of the movement
typedef enum { LEFT = 1, RIGHT = -1 } TURN_DIR;

// Function prototypes
int followUntilCross(TURN_DIR searchDir);
void followUntilCount(TURN_DIR searchDir, int encoderCount);
void followUntilDist(TURN_DIR searchDir, float distance);
void turnOnCross(TURN_DIR turnDir);

void setup() {
  // Setup code here

  // Set up the chassis
  chassis.init();
  chassis.idle();

  // Set up the rangefinder
  rangefinder.init();

  // Delay to allow user to get away
  delay(1000);

  // 30 degree turn to the right to get off line
  chassis.turnFor(-30, TURN_SPEED, true);

  // Line follow until we find the cross
  int firstCount = followUntilCross(RIGHT);

  // Move to next following line
  turnOnCross(RIGHT);

  // Line follow until we're 5 inches away from the wall
  followUntilDist(RIGHT, 5);

  // Delay to measure
  delay(3000);

  // Turn off the line
  chassis.turnFor(-30, TURN_SPEED, true);

  // Follow the line until the cross
  followUntilCross(RIGHT);

  // Move to next following line
  turnOnCross(LEFT);

  // Follow the line for the distance we moved before
  followUntilCount(LEFT, firstCount);
}

void loop() {
  // Repeating code here
}

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