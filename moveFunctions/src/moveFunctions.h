#pragma once

// Op Settings
#define FORWARD_SPEED 12   // in/s
#define TURN_SPEED 90      // deg/s
#define SEARCH_EFFORT 100  // Motor power, 0-300ish

// Type definitions
// Direction inverts the sign of the movement
typedef enum { LEFT = 1, RIGHT = -1 } TURN_DIR;

// Function prototypes
int followUntilCross(TURN_DIR searchDir);
void followUntilCount(TURN_DIR searchDir, int encoderCount);
void followUntilDist(TURN_DIR searchDir, float distance);
void turnOnCross(TURN_DIR turnDir);
int driveUntilCross();
void driveUntilDist(float distance);

#include "moveFunctions.cpp"