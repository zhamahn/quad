#include <Arduino.h>

#include "esc.h"

ESC::ESC(unsigned char _pin, float _pitch, float _roll, float _yaw, float _altitude) {
  pin             = _pin;
  pitch_factor    = _pitch;
  roll_factor     = _roll;
  yaw_factor      = _yaw;
  altitude_factor = _altitude;
}

void ESC::write(void) {
  analogWrite(pin, output);
}

int ESC::set(int value) {
  output = constrain(value, OUTPUT_MIN, OUTPUT_MAX);
  return output;
}

int ESC::change(int amount) { return set(output + amount); }

int ESC::changePitch(int amount)    { return change(amount * pitch_factor); }
int ESC::changeRoll(int amount)     { return change(amount * roll_factor); }
int ESC::changeYaw(int amount)      { return change(amount * yaw_factor); }
int ESC::changeAltitude(int amount) { return change(amount * altitude_factor); }
