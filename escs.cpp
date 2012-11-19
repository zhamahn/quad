#include <Arduino.h>

#include "quad.h"
#include "escs.h"

void ESCs::write(void) {
  x->write();
  nx->write();
  y->write();
  ny->write();
}

void ESCs::write(int value) {
  x->write(value);
  nx->write(value);
  y->write(value);
  ny->write(value);
}

void ESCs::decrease(int step) {
  x->decrease(step);
  nx->decrease(step);
  y->decrease(step);
  ny->decrease(step);
}

void ESCs::increase(int step) {
  x->increase(step);
  nx->increase(step);
  y->increase(step);
  ny->increase(step);
}

void ESCs::setGains(void) {
  double avg = average();
  x->gain  = x->output  - avg;
  nx->gain = nx->output - avg;
  y->gain  = y->output  - avg;
  ny->gain = ny->output - avg;
}

bool ESCs::allStopped(void) {
  return (x->stopped() && nx->stopped() && y->stopped() && ny->stopped());
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
