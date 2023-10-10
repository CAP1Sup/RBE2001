#pragma once

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