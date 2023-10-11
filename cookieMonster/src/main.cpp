#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <IRProcessor.h>
#include <Rangefinder.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

// Op Settings
// Overall section enable
#define ENABLE_OLD_PLATE_PICKUP
#define ENABLE_NEW_PLATE_DROP_OFF
#define ENABLE_MIDFIELD_PLATE_PICKUP
#define ENABLE_MIDFIELD_PLATE_DROP_OFF

// Chassis
#define FORWARD_SPEED 6            // in/s
#define HOUSE_GO_AROUND_DIST 12    // in
#define HOUSE_SIDE_TRAVEL_DIST 18  // in
#define BACKUP_SPEED 3             // in/s
#define BACKUP_DIST 3              // in
#define MIDFIELD_BACKUP_DIST 1     // in
#define TURNAROUND_ANGLE 160       // deg
#define TURN_SPEED 60              // deg/s
#define SEARCH_EFFORT 100          // Motor power, 0-300ish

// Line following
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800
#define LINE_FOLLOW_P 0.1    // deg/s per difference in sensor values

// Ultrasonic distances
#define STAGING_US_DIST 1           // in
#define HOUSE_25_US_MOVE_IN_DIST 2  // in
#define HOUSE_25_US_DIST 1          // in
#define HOUSE_45_US_MOVE_IN_DIST 3  // in
#define HOUSE_45_US_DIST 2          // in
#define MIDFIELD_US_DIST 2  // in, should be more than staging block dist

// 4 Bar
#define MANUAL_MOVE_EFFORT 100               // % of max effort
#define ENCODER_SAMPLING_TIME 100            // ms
#define STAGING_PLATFORM_ANGLE -19.1         // deg
#define HOUSE_45_DEG_PANEL_MOVE_IN_ANGLE 35  // deg
#define HOUSE_45_DEG_PANEL_ANGLE 41.78       // deg
#define HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE 55  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 75          // deg
#define CLEARANCE_ANGLE 70                   // deg
#define STAGING_CLEARANCE_ANGLE 0            // deg

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
RotaryGripper gripper(GRIPPER_FEEDBACK_PIN);
BlueMotor blueMotor(0, 1);
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;

// Variables
volatile int16_t irCode = -1;
volatile FIELD_SIDE fieldSide = UNKNOWN_SIDE;
volatile bool skipToMidfield = false;
volatile bool eStop = false;
volatile bool waitingForConfirm = false;

// Import the move functions
// Must be done after #defines and Chassis creation
#include <moveFunctions.h>

// Function prototypes
void processIRPress();

// Convenience function
void waitForConfirmation() {
  delay(3000);
  /*waitingForConfirm = true;
  while (true) {
    noInterrupts();
    if (!waitingForConfirm) {
      break;
    }
    interrupts();
    delay(10);
  }*/
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
  // uint32_t startTime = millis();
  // while (!Serial && (millis() - startTime) < 1000) {
  //  delay(10);
  // }

  // Open the gripper
  while (!gripper.setDesiredState(OPEN))
    ;

  // Wait for the user to select a field side
  // 1 = left, 3 = right
  while (true) {
    if (fieldSide != UNKNOWN_SIDE) {
      break;
    }
    if (buttonA.isPressed() || irCode == REMOTE_DOWN) {
      blueMotor.setEffort(-MANUAL_MOVE_EFFORT);
    } else if (buttonC.isPressed() || irCode == REMOTE_UP) {
      blueMotor.setEffort(MANUAL_MOVE_EFFORT);
    } else {
      blueMotor.setEffort(0);
    }
    delay(10);
  }

  if (!skipToMidfield) {
// Old panel grab
#ifdef ENABLE_OLD_PLATE_PICKUP
    chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
    followUntilCross(LEFT);
    turnOnCross((TURN_DIR)fieldSide);
    if (fieldSide == RIGHT_SIDE) {
      while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE))
        ;
      followUntilDist((TURN_DIR)fieldSide, HOUSE_25_US_MOVE_IN_DIST);
      while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_ANGLE))
        ;
      followUntilDist((TURN_DIR)fieldSide, HOUSE_25_US_DIST);
    } else {
      while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_MOVE_IN_ANGLE))
        ;
      followUntilDist((TURN_DIR)fieldSide, HOUSE_45_US_MOVE_IN_DIST);
      while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_ANGLE))
        ;
      followUntilDist((TURN_DIR)fieldSide, HOUSE_45_US_DIST);
    }
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
#endif
#ifdef ENABLE_NEW_PLATE_DROP_OFF
    // New panel placement
    while (!gripper.setDesiredState(CLOSED))
      ;
    while (!blueMotor.moveTo(CLEARANCE_ANGLE))
      ;
    chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
    followUntilCross(LEFT);
    turnOnCross((TURN_DIR)fieldSide);
    if (fieldSide == RIGHT_SIDE) {
      followUntilDist((TURN_DIR)fieldSide, HOUSE_25_US_DIST);
      while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_ANGLE))
        ;
    } else {
      followUntilDist((TURN_DIR)fieldSide, HOUSE_45_US_DIST);
      while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_ANGLE))
        ;
    }
    while (!gripper.setDesiredState(OPEN))
      ;
    // waitForConfirmation();

    // Move to other side of field
    chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                     true);
    while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
      ;
    chassis.turnFor(90 * fieldSide, TURN_SPEED, true);
    chassis.driveFor(HOUSE_GO_AROUND_DIST * INCHES_TO_CM,
                     FORWARD_SPEED * INCHES_TO_CM, true);
    chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
    chassis.driveFor(HOUSE_SIDE_TRAVEL_DIST * INCHES_TO_CM,
                     FORWARD_SPEED * INCHES_TO_CM, true);
    chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
    driveUntilCross();
    turnOnCross((TURN_DIR)-fieldSide);
    // waitForConfirmation();
#endif
  } else {
#ifdef ENABLE_MIDFIELD_PLATE_PICKUP
    // Old panel grab from midfield (from 2nd robot)
    driveUntilDist(MIDFIELD_US_DIST);
    followUntilDist(LEFT, STAGING_US_DIST);
    while (!gripper.setDesiredState(CLOSED))
      ;
    while (!blueMotor.moveTo(STAGING_CLEARANCE_ANGLE))
      ;
    chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                     true);
    chassis.turnFor(180, TURN_SPEED, true);
    driveUntilCross();
    turnOnCross((TURN_DIR)-fieldSide);
    followUntilDist((TURN_DIR)-fieldSide, STAGING_US_DIST);
    while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
      ;
    while (!gripper.setDesiredState(OPEN))
      ;
    waitForConfirmation();
#endif
#ifdef ENABLE_MIDFIELD_PLATE_DROP_OFF
    // New panel placement to midfield (for 2nd robot)
    while (!gripper.setDesiredState(CLOSED))
      ;
    while (!blueMotor.moveTo(STAGING_CLEARANCE_ANGLE))
      ;
    chassis.driveFor(-MIDFIELD_BACKUP_DIST * INCHES_TO_CM,
                     BACKUP_SPEED * INCHES_TO_CM, true);
    chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
    driveUntilDist(MIDFIELD_US_DIST);
    followUntilDist(LEFT, STAGING_US_DIST);
    while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
      ;
    while (!gripper.setDesiredState(OPEN))
      ;
    chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                     true);
    // Finished!
#endif
  }
}

void loop() {}

/**
 * @brief Processes the IR press
 *
 */
void processIRPress() {
  // Disable the interrupts
  noInterrupts();

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
    eStop = !eStop;
    chassis.setEStop(eStop);
    blueMotor.setEStop(eStop);
    gripper.setEStop(eStop);

    // Field side settings
  } else if (keyCode == REMOTE_1) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = LEFT_SIDE;
    }
  } else if (keyCode == REMOTE_2) {
    skipToMidfield = true;
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

  // Re-enable the interrupts
  interrupts();
}