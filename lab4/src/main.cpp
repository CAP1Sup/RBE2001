#include <Arduino.h>
#include <BlueMotor.h>
#include <Chassis.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#define MOTOR_EFFORT 50            // % of max effort
#define ENCODER_SAMPLING_TIME 100  // ms
#define ENCODER_TICKS_PER_REV 540  // ticks per revolution

// Create the objects
Chassis chassis;
RotaryGripper gripper(A4, 223, 355, 10, 16, 140);
BlueMotor motor(0, 1);
Romi32U4ButtonA buttonA;
Romi32U4ButtonC buttonC;

// Variables
unsigned long lastTime = 0;
int32_t lastCount = 0;

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

  // Initialize serial and wait for connection
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  // Open the gripper
  gripper.setDesiredState(OPEN);
}

void loop() {
  // Repeating code here
  /*
  if (buttonA.isPressed()) {
    motor.setEffort(MOTOR_EFFORT);
  } else if (buttonC.isPressed()) {
    motor.setEffort(-MOTOR_EFFORT);
  } else {
    motor.setEffort(0);
  }
  */

  // Blue motor testing (only uncomment one at a time)
  // Set these values in BlueMotor.h after testing
  // Positive
  for (int effort = 0; effort <= 100; effort += 1) {
    motor.setEffort(effort);
    uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
      Serial.print("Effort: ");
      Serial.print(effort);
      Serial.print(", Count: ");
      Serial.println(motor.getPosition());
      delay(10);
    }
  }

  // Negative
  /*
  for (int effort = 0; effort >= -100; effort -= 1) {
    motor.setEffort(effort);
    uint32_t startTime = millis();
    while (millis() - startTime < 1000) {
      Serial.print("Effort: ");
      Serial.print(effort);
      Serial.print(", Count: ");
      Serial.println(motor.getPosition());
      delay(10);
    }
  }*/
}