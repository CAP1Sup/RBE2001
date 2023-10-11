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
#define FIRST_ENABLE_OLD_PLATE_PICKUP
#define FIRST_ENABLE_NEW_PLATE_DROP_OFF
#define FIRST_ENABLE_MIDFIELD_PLATE_PICKUP
#define FIRST_ENABLE_MIDFIELD_PLATE_DROP_OFF
#define SECOND_ENABLE_OLD_PLATE_PICKUP
#define SECOND_ENABLE_NEW_PLATE_DROP_OFF

// Chassis
#define FORWARD_SPEED 3                // in/s
#define HOUSE_GO_AROUND_DIST 16        // in
#define HOUSE_SIDE_TRAVEL_DIST 22      // in
#define MIDFIELD_DIST_FROM_LINE 8.5    // in
#define MIDFIELD_DIST_BEFORE_US 2      // in
#define BACKUP_SPEED 1.5               // in/s
#define BACKUP_DIST 4                  // in
#define FIRST_MIDFIELD_BACKUP_DIST 1   // in
#define SECOND_MIDFIELD_BACKUP_DIST 4  // in
#define TURNAROUND_ANGLE 160           // deg
#define TURN_SPEED 30                  // deg/s
#define SEARCH_EFFORT 50               // Motor power, 0-300ish
#define END_SPEED 3                    // in/s
#define END_MOVE_COUNT 1440 / 3        // Encoder counts

// Line following
#define BLACK_THRESHOLD 500  // White: ~40, Black: ~800
#define LINE_FOLLOW_P 0.1    // deg/s per difference in sensor values

// Ultrasonic distances
#define STAGING_US_DIST 1.5              // in
#define HOUSE_25_US_MOVE_IN_DIST 3.5     // in
#define HOUSE_25_US_MOVE_IN_DIST_2 2.75  // in
#define HOUSE_25_US_DIST 1.75            // in
#define HOUSE_45_US_MOVE_IN_DIST 3       // in
#define HOUSE_45_US_DIST 2.5             // in
#define FIRST_MIDFIELD_US_DIST \
  3.5  // in, should be more than staging block dist
#define SECOND_MIDFIELD_US_DIST 1.5  // in
#define MIDFIELD_RAM_US_DIST \
  1  // in, should be less than second midfield
     // dist

// 4 Bar
#define MANUAL_MOVE_EFFORT 100                 // % of max effort
#define ENCODER_SAMPLING_TIME 100              // ms
#define STAGING_PLATFORM_ANGLE -19.06          // deg
#define HOUSE_45_DEG_PANEL_MOVE_IN_ANGLE 30    // deg
#define HOUSE_45_DEG_PANEL_ANGLE 36.78         // deg
#define HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE 40    // deg
#define HOUSE_25_DEG_PANEL_MOVE_IN_ANGLE_2 50  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 61.69         // deg
#define CLEARANCE_ANGLE 65                     // deg
#define BALANCE_ANGLE 80                       // deg
#define STAGING_CLEARANCE_ANGLE 0              // deg

// IR Codes
#define E_STOP REMOTE_VOL_PLUS
#define CONFIRM REMOTE_STOP_MODE

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
typedef enum { FIRST, SECOND, UNKNOWN_NUM } ROBOT_NUM;
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
volatile int16_t irCode = -1;
volatile ROBOT_NUM robotNum = UNKNOWN_NUM;
volatile FIELD_SIDE fieldSide = UNKNOWN_SIDE;
volatile bool skipToMidfield = false;
volatile bool eStop = false;
volatile bool waitingForConfirm = false;

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
    if (!waitingForConfirm) {
      break;
    }
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
    if (robotNum != UNKNOWN_NUM) {
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

  if (robotNum == FIRST) {
    if (!skipToMidfield) {
// Old panel grab
#ifdef FIRST_ENABLE_OLD_PLATE_PICKUP
      chassis.turnFor(TURNAROUND_ANGLE, TURN_SPEED, true);
      followUntilCross(LEFT);
      turnOnCross((TURN_DIR)fieldSide);
      raise4Bar();
      while (!gripper.setDesiredState(CLOSED))
        ;
      waitForConfirmation();  // to make sure the plate is in the gripper

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
      waitForConfirmation();  // to make sure the plate is placed correctly
      while (!gripper.setDesiredState(OPEN))
        ;
      waitForConfirmation();  // to make sure that the new plate is placed
#endif
#ifdef FIRST_ENABLE_NEW_PLATE_DROP_OFF
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
      waitForConfirmation();  // to make sure that the plate is placed correctly
      while (!gripper.setDesiredState(OPEN))
        ;

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
      waitForConfirmation();
#endif
    } else {
#ifdef FIRST_ENABLE_MIDFIELD_PLATE_PICKUP
      // Old panel grab from midfield (from 2nd robot)
      driveUntilDist(FIRST_MIDFIELD_US_DIST);
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
      waitForConfirmation();  // that the plates have been swapped
#endif
#ifdef FIRST_ENABLE_MIDFIELD_PLATE_DROP_OFF
      // New panel placement to midfield (for 2nd robot)
      while (!gripper.setDesiredState(CLOSED))
        ;
      while (!blueMotor.moveTo(STAGING_CLEARANCE_ANGLE))
        ;
      chassis.driveFor(-FIRST_MIDFIELD_BACKUP_DIST * INCHES_TO_CM,
                       BACKUP_SPEED * INCHES_TO_CM, true);
      chassis.turnFor(90 * -fieldSide, TURN_SPEED, true);
      driveUntilDist(FIRST_MIDFIELD_US_DIST);
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
  } else {
    // Robot must be second
    // Old panel grab
#ifdef SECOND_ENABLE_OLD_PLATE_PICKUP
    raise4Bar();
    while (!gripper.setDesiredState(CLOSED))
      ;
    waitForConfirmation();  // to make sure the plate is in the gripper

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
    followUntilDist((TURN_DIR)-fieldSide, SECOND_MIDFIELD_US_DIST);
    while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE))
      ;
    waitForConfirmation();  // to make sure the plate is placed correctly
    while (!gripper.setDesiredState(OPEN))
      ;
    waitForConfirmation();  // to make sure that the new plate is placed
#endif
#ifdef SECOND_ENABLE_NEW_PLATE_DROP_OFF
    // New panel placement
    driveUntilDist(MIDFIELD_RAM_US_DIST);
    while (!gripper.setDesiredState(CLOSED))
      ;
    while (!blueMotor.moveTo(BALANCE_ANGLE))
      ;
    chassis.driveFor(-SECOND_MIDFIELD_BACKUP_DIST * INCHES_TO_CM,
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
    waitForConfirmation();  // to make sure that the plate is placed correctly
    while (!gripper.setDesiredState(OPEN))
      ;
    // Start backing up
    chassis.getLeftEncoderCount(true);
    chassis.setWheelSpeeds(-END_SPEED * INCHES_TO_CM,
                           -END_SPEED * INCHES_TO_CM);
    while (!blueMotor.moveTo(STAGING_PLATFORM_ANGLE)) {
      if (chassis.getLeftEncoderCount(false) < -END_MOVE_COUNT) {
        chassis.idle();
      }
    }
    chassis.idle();
    // waitForConfirmation();
#endif
  }
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
    eStop = !eStop;
    chassis.setEStop(eStop);
    blueMotor.setEStop(eStop);
    gripper.setEStop(eStop);

    // Field settings
  } else if (keyCode == REMOTE_4) {
    if (robotNum == UNKNOWN_NUM) {
      robotNum = FIRST;
    }
  } else if (keyCode == REMOTE_5) {
    if (robotNum == UNKNOWN_NUM) {
      robotNum = SECOND;
    }
  } else if (keyCode == REMOTE_LEFT) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = LEFT_SIDE;
    }
  } else if (keyCode == REMOTE_RIGHT) {
    if (fieldSide == UNKNOWN_SIDE) {
      fieldSide = RIGHT_SIDE;
    }
  } else if (keyCode == REMOTE_BACK) {
    skipToMidfield = true;

    // Confirmation button
  } else if (keyCode == CONFIRM) {
    waitingForConfirm = false;

    // Other buttons
  } else {
    irCode = keyCode;
  }
}

/**
 * @brief Raises the 4 bar to the house side
 *
 */
void raise4Bar() {
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
}