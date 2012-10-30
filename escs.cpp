#include <Arduino.h>
#include "main.h"
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

void ESCs::setCorrections(void) {
  double avg = avgOutput();
  x->correction  = x->output  - avg;
  nx->correction = nx->output - avg;
  y->correction  = y->output  - avg;
  ny->correction = ny->output - avg;
}

bool ESCs::allStopped(void) {
  return (x->stopped() && nx->stopped() && y->stopped() && ny->stopped());
}

double ESCs::avg(void) {
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
