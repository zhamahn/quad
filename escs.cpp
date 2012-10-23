#include <Arduino.h>
#include "main.h"
#include "escs.h"

void ESCs::computePIDs(void) {
  x->pid->Compute();
  nx->pid->Compute();
  y->pid->Compute();
  nx->pid->Compute();
}

void ESCs::setOutputs(void) {
  x->set();
  nx->set();
  y->set();
  ny->set();
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
