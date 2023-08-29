// Written by: Christian Piper
// Date: 8/29/2023

#include <Arduino.h>
#include <wpi-32u4-lib.h>

// Declare a chassis object with nominal dimensions
// In practice, adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis;

#define MOVE_SPEED 12  // Inches per second
#define TURN_SPEED 90  // Degrees per second
#define IN_TO_CM 2.54

/**
 * @brief Drive in a polygon
 * @param sides Number of sides
 * @param length Length of each side in inches
 */
void polygon(int sides, float length) {
  float angle = 360.0f / sides;
  for (int sideIndex = 0; sideIndex < sides; sideIndex++) {
    chassis.driveFor(length * IN_TO_CM, MOVE_SPEED * IN_TO_CM, true);
    chassis.turnFor(angle, TURN_SPEED, true);
  }
}

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() {
  // initialize the chassis (which also initializes the motors)
  chassis.init();
  chassis.idle();

  // delay so the user can get away
  delay(3000);

  // these can be undone for the student to adjust
  // chassis.setMotorPIDcoeffs(5, 0.5);

  // Drive in a polygon
  polygon(3, 12);
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop() {}
