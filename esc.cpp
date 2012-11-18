#include <Arduino.h>

#include "esc.h"
#include "quad.h"

ESC::ESC(int _pin) {
  pin = _pin;
  correction = 0;
}

int ESC::write(void) {
  analogWrite(pin, output + correction);
  return output;
}

int ESC::write(int _output) {
  if (_output < OUTPUT_MIN)
    output = OUTPUT_MIN;
  else if (_output > OUTPUT_MAX)
    output = OUTPUT_MAX;
  else
    output = _output;

  write();
  return output;
}

int ESC::increase(void) {
  return increase(ESC_STEP);
}
int ESC::increase(int step) {
  return write(output + step);
}
int ESC::decrease(void) {
  return decrease(ESC_STEP);
}
int ESC::decrease(int step = ESC_STEP) {
  return change(output - step);
}
bool ESC::stopped(void) {
  return (output <= OUTPUT_MIN);
}
int ESC::change(int amount) {
  write(output + amount);
}
