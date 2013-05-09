#include <Arduino.h>
#include <SoftwareSerial.h>

#include "controller.h"

void Controller::updateButtons(void) {
  btn_stick_right = data[13] & B00000001;
  btn_stick_left  = data[13] & B00000010;
  btn_back        = data[13] & B00000100;
  btn_start       = data[13] & B00001000;
  btn_up          = data[13] & B00010000;
  btn_down        = data[13] & B00100000;
  btn_left        = data[13] & B01000000;
  btn_right       = data[13] & B10000000;
}

void Controller::updateFromDataArray(void) {
  stick_right_x  = data[0]<<8;
  stick_right_x |= data[1];

  stick_right_y  = data[2]<<8;
  stick_right_y |= data[3];

  stick_left_x  = data[4]<<8;
  stick_left_x |= data[5];

  stick_left_y  = data[6]<<8;
  stick_left_y |= data[7];

  trigger_right  = data[8]<<8;
  trigger_right |= data[9];

  trigger_left  = data[10]<<8;
  trigger_left |= data[11];

  updateButtons();
}

void Controller::reset(void) {
  stick_right_x = 0;
  stick_right_y = 0;
  stick_left_x = 0;
  stick_left_y = 0;
  trigger_right = 0;
  trigger_left = 0;
}

void Controller::update(void) {
  unsigned long now = millis();

  if (mySerial->available() > 0) {
    if (readFrame() == FRAME_COMPLETE)
      updateFromDataArray();
      lastUpdateAt = now;
  }
  if ((lastUpdateAt + 5000) < now)
    reset();
}

int Controller::countOnes(unsigned char data) {
  int ones = 0;
  unsigned char mask = 0x01;
  while (mask != 0) // Stays zero until bitshifted to zero
  {
    if (mask & data) { ones++; }
    mask = mask << 1; // Go to next bit
  }

  return ones;
}

int Controller::readFrame(void) {
  static char headersRead = 0;
  static int dataPosition = 0;
  static int ones = 0;
  unsigned char serialByte;
  int returnValue = FRAME_INCOMPLETE;
  int i;

  while (mySerial->available() > 0)
  {
    serialByte = mySerial->read();

    // Handle frame header
    if (headersRead < 1 && serialByte == FRAME_HEADER1) {
      headersRead += 1;
      returnValue = FRAME_INCOMPLETE;
    } else if (headersRead < 2 && serialByte == FRAME_HEADER2) {
      headersRead += 1;
      dataPosition = 0;
      for (i=0; i<CONTROLLER_DATA_ARRAY_LENGTH; i++)
        data[i] = 0;
      returnValue = FRAME_INCOMPLETE;
    // Handle frame data
    } else if (dataPosition < CONTROLLER_DATA_ARRAY_LENGTH) {
      data[dataPosition] = serialByte;
      ones += countOnes(serialByte);
      dataPosition++;
      returnValue = FRAME_INCOMPLETE;

    // Handle frame footer
    } else {
      // If current byte is frame footer and parity checks out
      if ( BYTE_IS_FRAME_FOOTER(serialByte) && (PARITY_BIT(serialByte) == (ones & 1)) ) {
        returnValue = FRAME_COMPLETE;
      } else {
        returnValue = FRAME_DISCARDED;
      }

      // Reset variables
      headersRead  = 0;
      dataPosition = 0;
      ones = 0;
    }
  }

  return returnValue;
}
