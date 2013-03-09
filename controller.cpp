#include <Arduino.h>
#include <SoftwareSerial.h>

#include "controller.h"
#include "quad.h"

void Controller::updateButtons(unsigned char data) {
  left_stick  = data & B10000000;
  right_stick = data & B1000000;
  back        = data & B100000;
  start       = data & B10000;
  a           = data & B1000;
  b           = data & B100;
  x           = data & B10;
  y           = data & B1;
}

void Controller::updateDpad(unsigned char data) {
  up    = data & B1000;
  down  = data & B100;
  left  = data & B10;
  right = data & B1;
}

void Controller::updateFromDataArray(unsigned char data[]) {
  right_trigger = data[0];
  right_stick_x = data[1];
  right_stick_y = data[2];
  left_trigger  = data[3];
  left_stick_x  = data[4];
  left_stick_y  = data[5];

  updateButtons(data[6]);
  updateDpad(data[7]);

  lastUpdateAt = millis();
}

#ifdef DEBUG
void Controller::print(HardwareSerial *serial) {
  serial->print("rt: "); serial->print(right_trigger, DEC); serial->print(", ");
  serial->print("lt: "); serial->print(left_trigger, DEC); serial->print(", ");

  serial->print("rsx: "); serial->print(right_stick_x, DEC); serial->print(", ");
  serial->print("rsy: "); serial->print(right_stick_y, DEC); serial->print(", ");

  serial->print("lsx: "); serial->print(left_stick_x, DEC); serial->print(", ");
  serial->print("lsy: "); serial->print(left_stick_y, DEC); serial->print(", ");

  /*serial->print("w: "); serial->print(white, DEC); serial->print(", ");*/
  /*serial->print("b: "); serial->print(black, DEC); serial->print(", ");*/
  /*serial->print("s: "); serial->print(start, DEC); serial->print(", ");*/
  /*serial->print("b: "); serial->print(back, DEC); serial->print(", ");*/
  /*serial->print("l: "); serial->print(left_stick, DEC); serial->print(", ");*/
  /*serial->print("r: "); serial->print(right_stick, DEC); serial->print(", ");*/

  /*serial->print("dpad (ulrd): ");*/
    /*serial->print(up, DEC);*/
    /*serial->print(left, DEC);*/
    /*serial->print(right, DEC);*/
    /*serial->print(down, DEC);*/

  serial->println("");
}
#endif

signed char Controller::pitch(void) {
  return map(left_stick_y, -128, 127, -45, 45);
}

signed char Controller::roll(void) {
  return map(left_stick_x, -128, 127, -45, 45);
}

int Controller::altitude(void) {
  return (right_trigger - left_trigger);
}

signed char Controller::yaw(void) {
  return map(right_stick_x, -127, 127, -45, 45);
}

void Controller::resetIfOldData(void) {
  if ((lastUpdateAt + 5000) < millis())
    reset();
}

void Controller::reset(void) {
  right_trigger = 10;
  left_trigger = 10;

  right_stick_x = 0;
  right_stick_y = 0;

  left_stick_x = 0;
  left_stick_y = 0;
}

void Controller::update(void) {
  unsigned char data[8];
  if (mySerial->available() > 0) {
    if (readFrame(data) == FRAME_COMPLETE)
      updateFromDataArray(data);
  }
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

int Controller::readFrame(unsigned char data[]) {
  static bool preambleRead = false;
  static int dataPosition = 0;
  static int ones = 0;
  byte serialByte;
  int returnValue = FRAME_INCOMPLETE;
  int i;

  while (mySerial->available() > 0)
  {
    serialByte = mySerial->read();

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
