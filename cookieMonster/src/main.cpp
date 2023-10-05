#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <Rangefinder.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#include "IRProcessor.h"

// Op Settings
// Chassis
#define FORWARD_SPEED 12   // in/s
#define TURN_SPEED 90      // deg/s
#define SEARCH_EFFORT 100  // Motor power, 0-300ish

// Line following
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800
#define LINE_FOLLOW_P 0.1    // deg/s per difference in sensor values

// 4 Bar
#define MANUAL_MOVE_EFFORT 50           // % of max effort
#define ENCODER_SAMPLING_TIME 100       // ms
#define STAGING_PLATFORM_ANGLE -19.1    // deg
#define HOUSE_45_DEG_PANEL_ANGLE 41.78  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 75.14  // deg
#define CLEARANCE_ANGLE 70              // deg

// IR Codes
#define E_STOP REMOTE_VOL_MINUS

// Conversions
#define INCHES_TO_CM 2.54

// Pin settings
#define IR_PIN 2
#define US_TRIG_PIN 3
#define US_ECHO_PIN 7
#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3
#define GRIPPER_FEEDBACK_PIN A4

// Create the objects
Chassis chassis;
IRProcessor irProcessor(IR_PIN);
Rangefinder rangefinder(US_ECHO_PIN, US_TRIG_PIN);
RotaryGripper gripper(A4);
BlueMotor motor(0, 1);
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;

// Type definitions
// Direction inverts the sign of the movement
typedef enum { LEFT = 1, RIGHT = -1 } TURN_DIR;

// Variables
int16_t irCode = -1;
int lVal, rVal;

// Function prototypes
int followUntilCross(TURN_DIR searchDir);
void followUntilCount(TURN_DIR searchDir, int encoderCount);
void followUntilDist(TURN_DIR searchDir, float distance);
void turnOnCross(TURN_DIR turnDir);
void processIRPress();

// Convenience function
void waitForButtonA() {
  while (!buttonA.isPressed())
    ;
}

void setup() {
  // Setup code here

  // Initialize the chassis
  // ! MUST BE DONE FOR BLUE MOTOR TO WORK
  chassis.init();
  chassis.idle();

  // Setup IR decoder
  irProcessor.init(processIRPress);

  // Setup the ultrasonic sensor
  rangefinder.init();

  // Initialize the gripper
  gripper.init();

  // Initialize the motor
  motor.init();

  // Turn off the motor
  motor.setEffort(0);

  // Set the motor's current angle
  // Should always be staging block
  motor.setAngle(STAGING_PLATFORM_ANGLE);

  // Set the motor's PID constants
  motor.setKp(15);
  motor.setKi(0);
  motor.setKd(2500);

  // Initialize serial and wait for connection
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  // Open the gripper
  gripper.setDesiredState(OPEN);
}

void loop() {
  // Print the IR code
  Serial.println(irCode);

  // Run the whatever the code says
  if (irCode == REMOTE_UP) {
    motor.setEffort(MANUAL_MOVE_EFFORT);
  } else if (irCode == REMOTE_DOWN) {
    motor.setEffort(-MANUAL_MOVE_EFFORT);
  } else {
    motor.setEffort(0);
  }
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

/**
 * @brief Processes the IR press
 *
 */
void processIRPress() {
  // Get the key code
  int16_t keyCode = irProcessor.getKeyCode();

  // Check if the code is valid
  if (keyCode == -1) {
    // Invalid code
    return;
  }

  // TODO: Process the code

  // Stop the motor if the e-stop button is pressed
  if (keyCode == E_STOP) {
    if (motor.isOverridden()) {
      motor.clearOverride();
    } else {
      motor.setOverride();
    }
  } else {
    irCode = keyCode;
  }
}