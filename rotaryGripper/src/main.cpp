#include <Arduino.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#define POT_PIN A4

// Create a gripper object
RotaryGripper gripper(POT_PIN, 223, 355, 10, 16, 140);

void setup() {
  // Setup code here

  Serial.begin(9600);
  while (!Serial)
    ;  // Wait for the serial connection to be established
  Serial.println("Initializing gripper");
  gripper.init();
}

void loop() {
  // Repeating code here
  Serial.println("Opening gripper");
  gripper.setDesiredState(OPEN);
  delay(1500);
  Serial.println("Closing gripper");
  gripper.setDesiredState(CLOSED);
  delay(3000);
}