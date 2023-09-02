#include <Arduino.h>
#include <Chassis.h>
#include <Romi32U4.h>

// Settings
#define FORWARD_SPEED 6   // in/s
#define TURN_SPEED 90     // deg/s
#define SEARCH_EFFORT 50  // Motor power, 0-300ish
#define INCHES_TO_CM 2.54
#define LINE_FOLLOW_P 0.1
#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3
#define BLACK_THRESHOLD 500

// Globals
Chassis chassis;
int lVal, rVal;
bool onLine;

// Type definitions
typedef enum { LEFT = 1, RIGHT = -1 } TURN_DIR;

// Function prototypes
int followUntilCross(TURN_DIR searchDir);
void followUntilCount(TURN_DIR searchDir, int encoderCount);
void turnOnCross(TURN_DIR turnDir);

void setup() {
  // Setup code here

  // Initialize the serial port
  // Serial.begin(9600);
  // Serial.println("Starting Romi Line Follower");

  // Set up the chassis
  chassis.init();
  chassis.idle();

  // Delay
  delay(1000);

  // 30 degree turn to the right to get off line
  chassis.turnFor(-30, TURN_SPEED, true);

  // Line follow until we find the cross
  int firstCount = followUntilCross(RIGHT);

  // Move to next following line
  turnOnCross(RIGHT);

  // Line follow for an encoder count
  followUntilCount(RIGHT, 1450);

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
 * Follows the line until the cross is found
 * @param driveDir The direction to drive while searching for the line
 * @param searchDir The direction to search for the line
 */
int followUntilCross(TURN_DIR searchDir) {
  // White: ~40
  // Black: ~800

  // We're not on the line yet
  onLine = false;

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
        // Add a set value to account for the distance between the line sensor
        // and the center of the robot
        return chassis.getLeftEncoderCount(true) + 375;

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
 * Follows the line until the given encoder count is reached
 * @param driveDir The direction to drive while searching for the line
 * @param searchDir The direction to search for the line
 * @param encoderCount The encoder count to stop at
 */
void followUntilCount(TURN_DIR searchDir, int encoderCount) {
  // White: ~40
  // Black: ~800

  // We're not on the line yet
  onLine = false;

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
 * Moves the robot forward over the cross and turns to the given direction by 70
 * deg
 */
void turnOnCross(TURN_DIR turnDir) {
  // Move forward over the cross
  chassis.driveFor(3 * INCHES_TO_CM, FORWARD_SPEED * INCHES_TO_CM, true);

  // Turn to the given direction
  chassis.turnFor(turnDir * 70, TURN_SPEED, true);
}