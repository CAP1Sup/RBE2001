#include <Arduino.h>
#include <Romi32U4.h>
#include <Chassis.h>

#define FORWARD_SPEED 3 // in/s
#define INCHES_TO_CM 2.54
#define LINE_FOLLOW_P 0.5
#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3

Chassis chassis;

void setup()
{
  // Setup code here

  // Set up the chassis
  chassis.init();
  chassis.idle();

  // Delay
  delay(3000);
}

void loop()
{
  // Repeating code here

  // Calculate the difference between the two line sensors
  int difference = analogRead(L_LINE_FOLLOW_PIN) - analogRead(R_LINE_FOLLOW_PIN);
  chassis.setTwist(FORWARD_SPEED * INCHES_TO_CM, LINE_FOLLOW_P * difference);
}