#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <IRProcessor.h>
#include <LinearGripper.h>
#include <Rangefinder.h>
#include <Romi32U4.h>

// Op Settings
// Overall section enable
// #define ENABLE_GRIPPER_TESTING
// #define ENABLE_US_TESTING
#define ENABLE_OLD_PLATE_PICKUP
#define ENABLE_NEW_PLATE_DROP_OFF

// Chassis
#define FORWARD_SPEED 3              // in/s
#define MIDFIELD_DIST_FROM_LINE 8.5  // in
#define MIDFIELD_DIST_BEFORE_US 2    // in
#define BACKUP_SPEED 1.5             // in/s
#define BACKUP_DIST 4                // in
#define MIDFIELD_BACKUP_DIST 4       // in
#define TURNAROUND_ANGLE 160         // deg
#define TURN_SPEED 30                // deg/s
#define SEARCH_EFFORT 100            // Motor power, 0-300ish
#define END_SPEED 3                  // in/s
#define END_MOVE_COUNT 1440 / 3      // Encoder counts

// Line following
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800
#define LINE_FOLLOW_P 0.1    // deg/s per difference in sensor values

// Ultrasonic distances
#define STAGING_US_DIST 1                // in
#define HOUSE_25_US_MOVE_IN_DIST 3.5     // in
#define HOUSE_25_US_MOVE_IN_DIST_2 2.75  // in
#define HOUSE_25_US_DIST 1.75            // in
#define HOUSE_45_US_MOVE_IN_DIST 3       // in
#define HOUSE_45_US_DIST 2.5             // in
#define MIDFIELD_US_DIST 1.5  // in, should be more than staging block dist

// 4 Bar
#define MANUAL_MOVE_EFFORT 100                 // % of max effort
#define ENCODER_SAMPLING_TIME 100              // ms
#define STAGING_PLATFORM_ANGLE -19.06          // deg
#define HOUSE_45_DEG_PANEL_MOVE_IN_ANGLE 25    // deg
#define HOUSE_45_DEG_PANEL_ANGLE 36.78         // deg
#define HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE 40    // deg
#define HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE_2 50  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 61.69         // deg
#define CLEARANCE_ANGLE 65                     // deg
#define BALANCE_ANGLE 80                       // deg
#define STAGING_CLEARANCE_ANGLE 0              // deg

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
LinearGripper gripper(GRIPPER_FEEDBACK_PIN);
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

#ifdef ENABLE_GRIPPER_TESTING
  while (true) {
    while (!gripper.setDesiredState(CLOSED))
      ;
    delay(1000);
    while (!gripper.setDesiredState(OPEN))
      ;
    delay(1000);
  }
#endif
#ifdef ENABLE_US_TESTING
  while (true) {
    Serial.println(rangefinder.getDistance() / INCHES_TO_CM);
    delay(100);
  }
#endif
// Old panel grab
#ifdef ENABLE_OLD_PLATE_PICKUP
  chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
  followUntilCross(LEFT);
  turnOnCross((TURN_DIR)fieldSide);
  if (fieldSide == RIGHT_SIDE) {
    while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE))
      ;
    followUntilDist((TURN_DIR)fieldSide, HOUSE_25_US_MOVE_IN_DIST);
    while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE_2))
      ;
    followUntilDist((TURN_DIR)fieldSide, HOUSE_25_US_MOVE_IN_DIST_2);
    while (!blueMotor.moveTo(HOUSE_25_DEG_PANEL_ANGLE))
      ;
    driveUntilDist(HOUSE_25_US_DIST);
  } else {
    while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_MOVE_IN_ANGLE))
      ;
    followUntilDist((TURN_DIR)fieldSide, HOUSE_45_US_MOVE_IN_DIST);
    while (!blueMotor.moveTo(HOUSE_45_DEG_PANEL_ANGLE))
      ;
    driveUntilDist(HOUSE_45_US_DIST);
  }
  while (!gripper.setDesiredState(CLOSED))
    ;

  // Old panel return
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
  chassis.driveFor(-BACKUP_DIST * INCHES_TO_CM, BACKUP_SPEED * INCHES_TO_CM,
                   true);
  while (!blueMotor.moveTo(BALANCE_ANGLE))
    ;
  chassis.turnFor(90 * fieldSide, TURN_SPEED, true);
  chassis.driveFor(MIDFIELD_DIST_FROM_LINE * INCHES_TO_CM,
                   FORWARD_SPEED * INCHES_TO_CM, true);
  chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
  chassis.driveFor(MIDFIELD_DIST_BEFORE_US * INCHES_TO_CM,
                   FORWARD_SPEED * INCHES_TO_CM, true);
  followUntilDist((TURN_DIR)-fieldSide, MIDFIELD_US_DIST);
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
  while (!blueMotor.moveTo(BALANCE_ANGLE))
    ;
  chassis.driveFor(-MIDFIELD_BACKUP_DIST * INCHES_TO_CM,
                   BACKUP_SPEED * INCHES_TO_CM, true);
  chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
  driveUntilCross();
  turnOnCross((TURN_DIR)fieldSide);
  while (!blueMotor.moveTo(CLEARANCE_ANGLE))
    ;
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
  // Start backing up
  chassis.getLeftEncoderCount(true);
  chassis.setWheelSpeeds(-END_SPEED * INCHES_TO_CM, -END_SPEED * INCHES_TO_CM);
  while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE)) {
    if (chassis.getLeftEncoderCount(false) < -END_MOVE_COUNT) {
      chassis.idle();
    }
  }
  chassis.idle();
  // waitForConfirmation();
#endif
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
    if (blueMotor.isOverridden()) {
      blueMotor.clearOverride();
    } else {
      blueMotor.setOverride();
    }

    // Field side settings
  } else if (keyCode == REMOTE_4) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = LEFT_SIDE;
    }
  } else if (keyCode == REMOTE_6) {
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