#include <Arduino.h>

#include "esc.h"

ESC::ESC(unsigned char _pin, float _pitch, float _roll, float _yaw, float _altitude) {
  pin             = _pin;
  pitch_factor    = _pitch;
  roll_factor     = _roll;
  yaw_factor      = _yaw;
  altitude_factor = _altitude;
}

int ESC::write(void) {
  output = constrain(output, OUTPUT_MIN, OUTPUT_MAX);
  analogWrite(pin, output);
  return output;
}

int ESC::change(int amount) {
  return output += amount;
}

int ESC::changePitch(int amount) {
  amount *= pitch_factor;
  if (amount > 0)
    amount = 0;
  return output += amount*PITCH_SCALE;
}

int ESC::changeRoll(int amount) {
  amount *= roll_factor;
  if (amount > 0)
    amount = 0;
  return output += amount*ROLL_SCALE;
}

int ESC::changeYaw(int amount) {
  return output += amount*yaw_factor*YAW_SCALE;
}

int ESC::changeAltitude(int amount) {
  return output += amount*altitude_factor*ALTITUDE_SCALE;
}
