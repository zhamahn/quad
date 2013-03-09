#include <Arduino.h>

#include "esc.h"

ESC::ESC(unsigned char _pin) {
  pin = _pin;
  gain = 0;
}

int ESC::write(void) {
  analogWrite(pin, output + gain);
  return output;
}

int ESC::set(int value) {
  output = constrain(value, OUTPUT_MIN, OUTPUT_MAX);
  return write();
}

int ESC::change(int amount) {
  return set(output + amount);
}
