#include "controller.h"
#include "main.h"
#include <Arduino.h>
#include <HardwareSerial.h>

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

  updateDeltas();
  updateButtons(data[6]);
  updateDpad(data[7]);

  lastUpdateAt = millis();
}

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

unsigned char Controller::axisDelta(unsigned char *axis) {
  if (axis == &left_trigger
      || axis == &right_trigger)
    return constrain(TRIGGER_ZERO - *axis, 0, 255);
  if (axis == &left_stick_y
      || axis == &left_stick_x
      || axis == &right_stick_y
      || axis == &left_stick_x)
    return STICK_ZERO - *axis;
}

void Controller::updateDeltas(void) {
  right_trigger_delta = axisDelta(&right_trigger);
  right_stick_x_delta = axisDelta(&right_stick_x);
  right_stick_y_delta = axisDelta(&right_stick_y);

  left_trigger_delta  = axisDelta(&left_trigger);
  left_stick_x_delta  = axisDelta(&left_stick_x);
  left_stick_y_delta  = axisDelta(&left_stick_y);
}
