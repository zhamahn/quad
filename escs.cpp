#include <Arduino.h>
#include "main.h"
#include "escs.h"

void ESCs::writeOutputs(void) {
  x->write();
  nx->write();
  y->write();
  ny->write();
}

void ESCs::decreaseOutputs(int step) {
  x->decrease(step);
  nx->decrease(step);
  y->decrease(step);
  ny->decrease(step);
}

void ESCs::increaseOutputs(int step) {
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

double ESCs::avgOutput(void) {
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
