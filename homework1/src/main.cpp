/**
 * Demonstration of controlling two servos simultanesouly. See documentation for
 * servo.h/.cpp for more details.
 *
 * */

#include <Arduino.h>
#include <Chassis.h>
#include <wpi-32u4-lib.h>

// Declare a chassis object with nominal dimensions
// In practice, adjust the parameters: wheel diam, encoder counts, wheel track
Chassis chassis(7.0, 1440, 14.9);

/*
 * This is the standard setup function that is called when the board is rebooted
 * It is used to initialize anything that needs to be done once.
 */
void setup() {
  // initialize the chassis (which also initializes the motors)
  chassis.init();
  chassis.idle();

  // these can be undone for the student to adjust
  //chassis.setMotorPIDcoeffs(5, 0.5);
}

/*
 * The main loop for the program. The loop function is repeatedly called
 * after setup() is complete.
 */
void loop() {

}
