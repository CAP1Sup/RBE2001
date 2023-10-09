#pragma once

#include <Arduino.h>

// Remote constants
#define REMOTE_ADDRESS_BYTE0 0x00
#define REMOTE_ADDRESS_BYTE1 0xBF

#define REMOTE_VOL_MINUS 0x00
#define REMOTE_PLAY_PAUSE 0x01
#define REMOTE_VOL_PLUS 0x02

#define REMOTE_SETUP 0x04
#define REMOTE_UP 0x05
#define REMOTE_STOP_MODE 0x06

#define REMOTE_LEFT 0x08
#define REMOTE_ENTER_SAVE 0x09
#define REMOTE_RIGHT 0x0A

#define REMOTE_0 0x0C
#define REMOTE_DOWN 0x0D
#define REMOTE_BACK 0x0E

#define REMOTE_1 0x10
#define REMOTE_2 0x11
#define REMOTE_3 0x12

#define REMOTE_4 0x14
#define REMOTE_5 0x15
#define REMOTE_6 0x16

#define REMOTE_7 0x18
#define REMOTE_8 0x19
#define REMOTE_9 0x1A

/** \class IRProcessor
 * A class to interpret IR remotes with NEC encoding.
 *
 * NEC encoding sends four bytes: [device ID, ~divice ID, key code, ~key code]
 *
 * Sending the inverse allow for easy error checking (and reduces saturation in
 * the receiver).
 *
 * Codes are send in little endian; this library reverses upon reception, so the
 * first bit received is in the LSB of currCode. That means that the key code is
 * found in bits [23..16] of currCode
 *
 * https://techdocs.altium.com/display/FPGA/NEC+Infrared+Transmission+Protocol
 *
 * This does not interpret the codes into which key was pressed. That needs to
 * be mapped on a remote by remote basis.
 */
class IRProcessor {
 private:
  uint8_t pin = -1;

  enum IR_STATE {
    IR_READY,     // idle, returns to this state after you request a code
    IR_PREAMBLE,  // received the start burst, waiting for first bit
    IR_REPEAT,    // received repeat code (part of NEC protocol); last code will
                  // be returned
    IR_ACTIVE,    // have some bits, but not yet complete
    IR_COMPLETE,  // a valid code has been received
    IR_ERROR      // an error occurred; won't return a valid code
  };

  IR_STATE state = IR_READY;  // a simple state machine for managing reception

  volatile uint32_t lastReceiveTime =
      0;  // not really used -- could be used to sunset codes

  volatile uint32_t currCode = 0;  // the most recently received valid code
  volatile uint8_t index = 0;      // for tracking which bit we're on

  volatile uint32_t fallingEdge = 0;
  volatile uint32_t risingEdge = 0;

  volatile uint32_t lastRisingEdge =
      0;  // used for tracking spacing between rising edges, i.e., bit value

  // Function to run when a code is received
  void (*processCodeFn)(void) = NULL;

 public:
  IRProcessor(uint8_t p) : pin(p) {}
  void init(void (*processCode)(void));  // call this in the setup()
  void handleIRsensor(void);             // ISR

  uint32_t getCode(void)  // returns the most recent valid code; returns zero if
                          // there was an error
  {
    if (state == IR_COMPLETE || state == IR_REPEAT) {
      state = IR_READY;
      return currCode;
    } else
      return 0;
  }

  int16_t getKeyCode(
      bool acceptRepeat =
          false)  // returns the most recent key code; returns -1 on error (not
                  // sure if 0 can be a code or not!!!)
  {
    if (state == IR_COMPLETE || (acceptRepeat == true && state == IR_REPEAT)) {
      state = IR_READY;
      return (currCode >> 16) & 0x0ff;
    } else
      return -1;
  }
};

extern IRProcessor irProcessor;

#include "IRProcessor.cpp"