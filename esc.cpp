#include "esc.h"
#include "main.h"
#include <Arduino.h>

ESC::ESC(int _pin) {
  pid = new PID(&input, &output, &setpoint, KP, KI, KD, AUTOMATIC);
  pid->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  pin = _pin;
  correction = 0;
}

double ESC::set(void) {
  analogWrite(pin, output + correction);
  return output;
}

double ESC::set(double _output) {
  if (_output < OUTPUT_MIN)
    output = OUTPUT_MIN;
  else if (_output > OUTPUT_MAX)
    output = OUTPUT_MAX;
  else
    output = output;

  analogWrite(pin, output + correction);
  return output;
}

double ESC::increase(void) {
  return increase(ESC_STEP);
}
double ESC::increase(int step) {
  return set(output + step);
}
double ESC::decrease(void) {
  return decrease(ESC_STEP);
}
double ESC::decrease(int step = ESC_STEP) {
  return set(output - step);
}
