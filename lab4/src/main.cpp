#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#define MOTOR_EFFORT 50                 // % of max effort
#define ENCODER_SAMPLING_TIME 100       // ms
#define STAGING_PLATFORM_ANGLE -19.1    // deg
#define HOUSE_45_DEG_PANEL_ANGLE 41.78  // deg
#define HOUSE_25_DEG_PANEL_ANGLE 75.14  // deg
#define CLEARANCE_ANGLE 70              // deg

// #define MANUAL_MOVE
// #define PID_TESTING

// Create the objects
Chassis chassis;
RotaryGripper gripper(A4);
BlueMotor motor(0, 1);
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;

// Variables
unsigned long lastTime = 0;
int32_t currentCount = 0;
int32_t lastCount = 0;

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
  // Serial.begin(9600);
  // while (!Serial) {
  //  delay(10);
  //}

  // Open the gripper
  gripper.setDesiredState(OPEN);

#if !(defined(MANUAL_MOVE) || defined(PID_TESTING))
  // 4 bar testing code
  waitForButtonA();
  while (!gripper.setDesiredState(CLOSED))
    ;
  waitForButtonA();
  while (!motor.moveTo(CLEARANCE_ANGLE))
    ;
  waitForButtonA();
  while (!motor.moveTo(HOUSE_45_DEG_PANEL_ANGLE))
    ;
  waitForButtonA();
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForButtonA();
  while (!motor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  waitForButtonA();
  while (!gripper.setDesiredState(CLOSED))
    ;
  waitForButtonA();
  while (!motor.moveTo(CLEARANCE_ANGLE))
    ;
  waitForButtonA();
  while (!motor.moveTo(HOUSE_25_DEG_PANEL_ANGLE))
    ;
  waitForButtonA();
  while (!gripper.setDesiredState(OPEN))
    ;
  waitForButtonA();
  while (!motor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
#endif

  // Blue motor testing (only uncomment one at a time)
  // Set these values in BlueMotor.h after testing
  // Positive
  /*
  for (int effort = 0; effort <= 100; effort += 1) {
    motor.setEffortDBC(effort);
    uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
      Serial.print("Ms: ");
      Serial.print(millis());
      Serial.print(", User Effort: ");
      Serial.print(effort*4);
      //Serial.print(" Count: ");
      //Serial.println(motor.getPosition());
      Serial.print(", Adj Effort: ");
      Serial.print(motor.calculateDBCEffort(effort)*4);
      Serial.print(", Ang Spd: ");
      currentCount = motor.getPosition();
      // Calculate the encoder RPM
      float rpm = (currentCount - lastCount) * 60000.0f / 100 /
                540;
      lastCount = currentCount;
      Serial.println(rpm * 2 * 3.14 / 60);

      delay(100);
    }
  }*/

  // Negative
  /*
  for (int effort = 0; effort >= -100; effort -= 1) {
    motor.setEffortDBC(effort);
    uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
      Serial.print("Ms: ");
      Serial.print(millis());
      Serial.print(", User Effort: ");
      Serial.print(effort*4);
      //Serial.print(" Count: ");
      //Serial.println(motor.getPosition());
      Serial.print(", Adj Effort: ");
      Serial.print(motor.calculateDBCEffort(effort)*4);
      Serial.print(", Ang Spd: ");
      currentCount = motor.getPosition();
      // Calculate the encoder RPM
      float rpm = (currentCount - lastCount) * 60000.0f / 100 /
                540;
      lastCount = currentCount;
      Serial.println(rpm * 2 * 3.14 / 60);

      delay(100);
    }
  }*/
}

void loop() {
#ifdef MANUAL_MOVE
  // Repeating code here
  if (buttonA.isPressed()) {
    motor.setEffort(MOTOR_EFFORT);
  } else if (buttonC.isPressed()) {
    motor.setEffort(-MOTOR_EFFORT);
  } else {
    motor.setEffort(0);
  }
#elif defined(PID_TESTING)
  // Move the motor back and forth between the staging and clearance positions
  while (!motor.moveTo(CLEARANCE_ANGLE))
    ;
  delay(2000);
  while (!motor.moveTo(STAGING_PLATFORM_ANGLE))
    ;
  delay(2000);
#endif
}