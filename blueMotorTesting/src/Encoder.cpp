#include "Encoder.h"

// Initialize the static variables
uint8_t Encoder::_pinA = 0;
uint8_t Encoder::_pinB = 0;
int32_t Encoder::position = 0;

// Constructor
Encoder::Encoder() {}

/**
 * @brief Initializes the encoder
 *
 * @param pinA The pin number for the first encoder pin
 * @param pinB The pin number for the second encoder pin
 */
void Encoder::init(uint8_t pinA, uint8_t pinB) {
  // Set the pin numbers
  _pinA = pinA;
  _pinB = pinB;

  // Initialize the position
  position = 0;

  // Set the pin modes
  pinMode(_pinA, INPUT_PULLUP);
  pinMode(_pinB, INPUT_PULLUP);

  // Attach the interrupts
  attachInterrupt(digitalPinToInterrupt(_pinA), Encoder::isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_pinB), Encoder::isrB, CHANGE);
}

/**
 * @brief Gets the current encoder position. Interrupt safe.
 *
 * @return int32_t The current encoder position
 */
int32_t Encoder::getPosition() {
  // Disable interrupts
  noInterrupts();

  // Get the position
  int32_t ret = Encoder::position;

  // Enable interrupts
  interrupts();

  // Return the position
  return ret;
}

/**
 * @brief Resets the encoder position to 0
 */
void Encoder::resetPosition() {
  // Reset the position
  position = 0;
}

/**
 * @brief Interrupt service routine for the first encoder pin
 */
void Encoder::isrA() {
  // Compare the pin values
  if (digitalRead(_pinA) == digitalRead(_pinB)) {
    // Increment the position
    position++;
  } else {
    // Decrement the position
    position--;
  }
}

/**
 * @brief Interrupt service routine for the second encoder pin
 */
void Encoder::isrB() {
  // Compare the pin values
  if (digitalRead(_pinA) == digitalRead(_pinB)) {
    // Decrement the position
    position--;
  } else {
    // Increment the position
    position++;
  }
}