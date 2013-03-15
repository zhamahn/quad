#include <Arduino.h>
#include <SoftwareSerial.h>

#include "controller.h"

void Controller::updateButtons(unsigned char data) {
  left_stick  = data & 0b10000000;
  right_stick = data & 0b1000000;
  back        = data & 0b100000;
  start       = data & 0b10000;
  a           = data & 0b1000;
  b           = data & 0b100;
  x           = data & 0b10;
  y           = data & 0b1;
}

void Controller::updateDpad(unsigned char data) {
  up    = data & 0b1000;
  down  = data & 0b100;
  left  = data & 0b10;
  right = data & 0b1;
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
void Controller::print(void) {
  Serial.print("rt: "); Serial.print(right_trigger, DEC); Serial.print(", ");
  Serial.print("lt: "); Serial.print(left_trigger, DEC); Serial.print(", ");

  Serial.print("rsx: "); Serial.print(right_stick_x, DEC); Serial.print(", ");
  Serial.print("rsy: "); Serial.print(right_stick_y, DEC); Serial.print(", ");

  Serial.print("lsx: "); Serial.print(left_stick_x, DEC); Serial.print(", ");
  Serial.print("lsy: "); Serial.print(left_stick_y, DEC); Serial.print(", ");

  /*Serial.print("w: "); Serial.print(white, DEC); Serial.print(", ");*/
  /*Serial.print("b: "); Serial.print(black, DEC); Serial.print(", ");*/
  /*Serial.print("s: "); Serial.print(start, DEC); Serial.print(", ");*/
  /*Serial.print("b: "); Serial.print(back, DEC); Serial.print(", ");*/
  /*Serial.print("l: "); Serial.print(left_stick, DEC); Serial.print(", ");*/
  /*Serial.print("r: "); Serial.print(right_stick, DEC); Serial.print(", ");*/

  /*Serial.print("dpad (ulrd): ");*/
    /*Serial.print(up, DEC);*/
    /*Serial.print(left, DEC);*/
    /*Serial.print(right, DEC);*/
    /*Serial.print(down, DEC);*/

  Serial.println("");
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
