#include "communication.h"
#include "main.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

int countOnes(unsigned char data) {
  int ones = 0;
  unsigned char mask = 0x01;
  while (mask != 0) // Stays zero until bitshifted to zero
  {
    if (mask & data) { ones++; }
    mask = mask << 1; // Go to next bit
  }

  return ones;
}

int readFrame(SoftwareSerial *serial, unsigned char data[]) {
  static bool preambleRead = false;
  static int dataPosition = 0;
  static int ones = 0;
  byte serialByte;
  int returnValue = FRAME_INCOMPLETE;
  int i;

  while (serial->available() > 0)
  {
    serialByte = serial->read();

    // Handle frame header
    if (!preambleRead) {
      if (BYTE_IS_FRAME_HEADER(serialByte)) {
        preambleRead = true;
        returnValue = FRAME_INCOMPLETE;
      }

    // Handle frame data
    } else if (dataPosition < FRAME_DATA_LENGTH) {
      data[dataPosition] = serialByte;
      ones += countOnes(serialByte);
      dataPosition++;
      returnValue = FRAME_INCOMPLETE;

    // Handle frame footer
    } else {
      // If current byte is frame footer and parity checks out
      if ( BYTE_IS_FRAME_FOOTER(serialByte) && (PARITY_BIT(serialByte) == (ones & 1)) ) {
        return FRAME_COMPLETE;
      } else {
        returnValue = FRAME_DISCARDED;
      }

      // Reset variables
      preambleRead = false;
      dataPosition = 0;
      ones = 0;
      for (i=0; i<FRAME_DATA_LENGTH; i++)
        data[i] = 0;
    }
  }

  return returnValue;
}
