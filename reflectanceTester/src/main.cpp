#include <Arduino.h>
#include <Romi32U4.h>

#define L_LINE_FOLLOW_PIN A2
#define R_LINE_FOLLOW_PIN A3

void setup() {
  // Setup code here
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }

  pinMode(L_LINE_FOLLOW_PIN, INPUT);
  pinMode(R_LINE_FOLLOW_PIN, INPUT);
}

void loop() {
  // Repeating code here
  Serial.print("Left: ");
  Serial.print(analogRead(L_LINE_FOLLOW_PIN));
  Serial.print(" Right: ");
  Serial.println(analogRead(R_LINE_FOLLOW_PIN));
}