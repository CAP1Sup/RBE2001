#include <Arduino.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#define POT_PIN A4

// Create a gripper object
RotaryGripper gripper(POT_PIN, 24, 140, 0, 34.4);

void setup() {
  // Setup code here

  Serial.begin(9600);
  while (!Serial)
    ;  // Wait for the serial connection to be established
  gripper.init();
}

void loop() {
  // Repeating code here
  gripper.setDesiredState(OPEN);
  delay(1500);
  gripper.setDesiredState(CLOSED);
  delay(1500);
}