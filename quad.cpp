#include <Arduino.h>
#include "quad.h"
#include "main.h"

Quad::Quad(void) {
  pitchPID = new PID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, AUTOMATIC);
  rollPID = new PID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, AUTOMATIC);

  pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void Quad::stabilize(void) {
  acc->read();
  while (!acc->stable()) {
    if (acc->y > STABILITY_THRESHOLD)      { escs->y->decrease(); escs->ny->increase(); }
    else if (acc->y < STABILITY_THRESHOLD) { escs->y->increase(); escs->ny->decrease(); }

    if (acc->x > STABILITY_THRESHOLD)      { escs->x->decrease(); escs->nx->increase(); }
    else if (acc->x < STABILITY_THRESHOLD) { escs->x->increase(); escs->nx->decrease(); }

    acc->read();
  }
}

void Quad::landNow(void) {
  stabilize();

  // Halt all motors
  while (! escs->allStopped()) {
    escs->decrease();
    delay(100);
  }
}

void Quad::preFlight(void) {
  debug("Running pre-flight setup");
  setAltitude(20);
  escs->setCorrections();
}

void Quad::computePIDs(void) {
  pitchInput = (double)(acc->pitch());
  rollInput = (double)(acc->roll());
  altitudeInput = (double)(alt->distance);

  pitchSetpoint = (double)(controller->pitch());
  rollSetpoint = (double)(controller->roll());
  altitudeSetpoint = (double)(controller->altitude(alt->distance));

  pitchPID->Compute();
  rollPID->Compute();
  altitudePID->Compute();
}

void Quad::setESCs(void) {
  escs->changePitch(pitchOutput);
  escs->changeRoll(rollOutput);
  escs->changeAltitude(altitudeOutput);
}
