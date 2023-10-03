#include <Arduino.h>
#include <Romi32U4.h>
#include <RotaryGripper.h>

#define POT_PIN A4

// Create a gripper object
RotaryGripper gripper(POT_PIN);

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
  while (!gripper.setDesiredState(OPEN))
    ;
  Serial.print("Gripper state: ");
  Serial.println(gripper.getCurrentState());
  delay(1500);
  Serial.println("Closing gripper");
  while (!gripper.setDesiredState(CLOSED))
    ;
  Serial.print("Gripper state: ");
  Serial.println(gripper.getCurrentState());
  delay(3000);
}