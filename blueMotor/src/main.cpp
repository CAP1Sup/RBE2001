#include <Arduino.h>
#include <BlueMotor.h>
#include <Romi32U4.h>

#define MOTOR_EFFORT 50            // % of max effort
#define ENCODER_SAMPLING_TIME 100  // ms
#define ENCODER_TICKS_PER_REV 540  // ticks per revolution

// Create the objects
BlueMotor motor(0, 1);
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;

// Variables
unsigned long lastTime = 0;
int32_t lastCount = 0;

void setup() {
  // Setup code here

  // Initialize the motor
  motor.init();

  // Turn off the motor
  motor.setEffort(0);

  // Initialize serial and wait for connection
  Serial.begin(9600);
  while (!Serial) {
    delay(10);
  }
}

void loop() {
  // Repeating code here
  if (buttonB.isPressed()) {
    motor.setEffort(MOTOR_EFFORT);
  } else if (buttonC.isPressed()) {
    motor.setEffort(-MOTOR_EFFORT);
  } else {
    motor.setEffort(0);
  }

  // Compare the current time to the last update time
  if (millis() - lastTime >= ENCODER_SAMPLING_TIME) {
    // Print the time
    Serial.print("T: ");
    Serial.print(millis());

    // Get the current encoder count
    int32_t currentCount = motor.getPosition();

    // Print the encoder count
    Serial.print(" C: ");
    Serial.print(currentCount);

    // Calculate the encoder RPM
    float rpm = (currentCount - lastCount) * 60000.0f / ENCODER_SAMPLING_TIME /
                ENCODER_TICKS_PER_REV;

    // Print the encoder RPM
    Serial.print(" RPM: ");
    Serial.println(rpm);

    // Update the last count
    lastCount = currentCount;

    // Update the last time
    lastTime = millis();
  }
}