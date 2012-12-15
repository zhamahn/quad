#include <Arduino.h>

#include "quad.h"
#include "escs.h"

void ESCs::write(void) {
  x->write();
  nx->write();
  y->write();
  ny->write();
}

void ESCs::set(int value) {
  x->set(value);
  nx->set(value);
  y->set(value);
  ny->set(value);
}

void ESCs::setGains(void) {
  double avg = average();
  x->gain  = x->output  - avg;
  nx->gain = nx->output - avg;
  y->gain  = y->output  - avg;
  ny->gain = ny->output - avg;
}

double ESCs::average(void) {
  return (x->output + nx->output + y->output + ny->output) / 4;
}

void ESCs::changePitch(int amount) {
  x->change(amount * -1);
  nx->change(amount);
}

void ESCs::changeRoll(int amount) {
  y->change(amount * -1);
  ny->change(amount);
}

void ESCs::changeAltitude(int amount) {
  x->change(amount);
  nx->change(amount);
  y->change(amount);
  ny->change(amount);
}
