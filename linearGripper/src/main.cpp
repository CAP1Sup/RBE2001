#include <Arduino.h>
#include <LinearGripper.h>
#include <Romi32U4.h>

#define POT_PIN A4

// Create a gripper object
LinearGripper gripper(POT_PIN, 223, 355);

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
  Serial.print("Gripper position: ");
  Serial.println(gripper.getPosition());
  /*
  Serial.println("Opening gripper");
  gripper.setDesiredState(OPEN);
  delay(1500);
  Serial.println("Closing gripper");
  gripper.setDesiredState(CLOSED);
  delay(3000);
  */
}