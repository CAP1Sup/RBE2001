#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <IRProcessor.h>
#include <Rangefinder.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

// Op Settings
// Chassis
#define FORWARD_SPEED 3            // in/s
#define HOUSE_GO_AROUND_DIST 5     // in
#define HOUSE_SIDE_TRAVEL_DIST 12  // in
#define BACKUP_SPEED 2             // in/s
#define BACKUP_DIST 2              // in
#define MIDFIELD_BACKUP_DIST 4     // in
#define TURNAROUND_ANGLE 160       // deg
#define TURN_SPEED 30              // deg/s
#define SEARCH_EFFORT 100          // Motor power, 0-300ish

// Line following
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800
#define LINE_FOLLOW_P 0.1    // deg/s per difference in sensor values

// Ultrasonic distances
#define STAGING_US_DIST 5    // in
#define HOUSE_US_DIST 12     // in
#define MIDFIELD_US_DIST 10  // in, should be more than staging block dist

// 4 Bar
#define MANUAL_MOVE_EFFORT 100          // % of max effort
#define ENCODER_SAMPLING_TIME 100       // ms
#define STAGING_PLATFORM_ANGLE -19.1    // deg
#define HOUSE_45_DEG_PANEL_ANGLE 41.78  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 75.14  // deg
#define CLEARANCE_ANGLE 70              // deg

// IR Codes
#define E_STOP REMOTE_VOL_MINUS
#define CONFIRM REMOTE_PLAY_PAUSE

// Conversions
#define INCHES_TO_CM 2.54

// Pin settings
#define IR_PIN 11
#define US_TRIG_PIN 12
#define US_ECHO_PIN 3
#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3
#define GRIPPER_FEEDBACK_PIN A4

// Type definitions
// Field side
typedef enum { LEFT_SIDE = 1, RIGHT_SIDE = -1, UNKNOWN_SIDE = 0 } FIELD_SIDE;

// Create the objects
Chassis chassis;
IRProcessor irProcessor(IR_PIN);
Rangefinder rangefinder(US_ECHO_PIN, US_TRIG_PIN);
RotaryGripper gripper(A4);
BlueMotor blueMotor(0, 1);
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;

// Variables
int16_t irCode = -1;
FIELD_SIDE fieldSide = UNKNOWN_SIDE;

bool waitingForConfirm = false;

// Import the move functions
// Must be done after #defines and Chassis creation
#include <moveFunctions.h>

// Function prototypes
void processIRPress();
void raise4Bar();

// Convenience function
void waitForConfirmation() {
  waitingForConfirm = true;
  while (true) {
    noInterrupts();
    if (!waitingForConfirm) {
      break;
    }
    interrupts();
    delay(10);
  }
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
  blueMotor.init();

  // Turn off the motor
  blueMotor.setEffort(0);

  // Set the motor's current angle
  // Should always be staging block
  blueMotor.setAngle(STAGING_PLATFORM_ANGLE);

  // Set the motor's PID constants
  blueMotor.setKp(15);
  blueMotor.setKi(0);
  blueMotor.setKd(2500);

  // Initialize serial and wait for connection
  Serial.begin(9600);
  uint32_t startTime = millis();
  while (!Serial && (millis() - startTime) < 5000) {
    delay(10);
  }

  // Open the gripper
  while (!gripper.setDesiredState(OPEN))
    ;

  // Wait for the user to select a field side
  // 1 = left, 3 = right
  while (true) {
    noInterrupts();
    if (fieldSide != UNKNOWN_SIDE) {
      break;
    }
    interrupts();
    if (buttonA.isPressed()) {
      blueMotor.setEffort(-MANUAL_MOVE_EFFORT);
    } else if (buttonC.isPressed()) {
      blueMotor.setEffort(MANUAL_MOVE_EFFORT);
    } else {
      blueMotor.setEffort(0);
    }
    if (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '1') {
        fieldSide = LEFT_SIDE;
      } else if (c == '3') {
        fieldSide = RIGHT_SIDE;
      }
    }
    delay(10);
  }

  delay(5000);

  // Old panel grab
  chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
  followUntilCross(LEFT);
  turnOnCross((TURN_DIR)fieldSide);
  raise4Bar();
  followUntilDist(LEFT, HOUSE_US_DIST);
  while (!gripper.setDesiredState(CLOSED))
    ;

  // Old panel return
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
  chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                   true);
  chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
  followUntilCross(LEFT);
  turnOnCross((TURN_DIR)-fieldSide);
  followUntilDist((TURN_DIR)-fieldSide, STAGING_US_DIST);
  while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForConfirmation();

  // New panel placement
  while (!gripper.setDesiredState(CLOSED))
    ;
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
  chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
  followUntilCross(LEFT);
  turnOnCross((TURN_DIR)fieldSide);
  followUntilDist((TURN_DIR)fieldSide, HOUSE_US_DIST);
  raise4Bar();
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForConfirmation();

  // Move to other side of field
  chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                   true);
  chassis.turnFor(90 * fieldSide, TURN_SPEED, true);
  chassis.driveFor(HOUSE_GO_AROUND_DIST * INCHES_TO_CM,
                   FORWARD_SPEED * INCHES_TO_CM, true);
  chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
  chassis.driveFor(HOUSE_SIDE_TRAVEL_DIST * INCHES_TO_CM,
                   FORWARD_SPEED * INCHES_TO_CM, true);
  chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
  driveUntilCross();
  turnOnCross((TURN_DIR)fieldSide);
  waitForConfirmation();

  // Old panel grab from midfield (from 2nd robot)
  while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  driveUntilDist(MIDFIELD_US_DIST);
  followUntilDist(LEFT, STAGING_US_DIST);
  while (!gripper.setDesiredState(CLOSED))
    ;
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
  chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                   true);
  chassis.turnFor(180, TURN_SPEED, true);
  driveUntilCross();
  turnOnCross((TURN_DIR)fieldSide);
  followUntilDist((TURN_DIR)fieldSide, STAGING_US_DIST);
  while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForConfirmation();

  // New panel placement to midfield (for 2nd robot)
  while (!gripper.setDesiredState(CLOSED))
    ;
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
  chassis.driveFor(-MIDFIELD_BACKUP_DIST * INCHES_TO_CM,
                   BACKUP_SPEED * INCHES_TO_CM, true);
  chassis.turnFor(90 * fieldSide, TURN_SPEED, true);
  driveUntilDist(MIDFIELD_US_DIST);
  followUntilDist(LEFT, STAGING_US_DIST);
  while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForConfirmation();
  chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                   true);
  // Finished!
}

void loop() {}

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
    if (blueMotor.isOverridden()) {
      blueMotor.clearOverride();
    } else {
      blueMotor.setOverride();
    }

    // Field side settings
  } else if (keyCode == REMOTE_1) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = LEFT_SIDE;
    }
  } else if (keyCode == REMOTE_3) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = RIGHT_SIDE;
    }

    // Confirmation button
  } else if (keyCode == CONFIRM) {
    waitingForConfirm = false;

    // Other buttons
  } else {
    irCode = keyCode;
  }
}

/**
 * @brief Raises the 4 bar to the correct height
 *
 */
void raise4Bar() {
  if (fieldSide == LEFT_SIDE) {
    // Raise the 4 bar to the 25 deg angle
    while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_ANGLE))
      ;
  } else if (fieldSide == RIGHT_SIDE) {
    // Raise the 4 bar to the 45 deg angle
    while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_ANGLE))
      ;
  }
}