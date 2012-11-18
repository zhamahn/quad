#include <Arduino.h>
#include "control_center.h"

ControlCenter::ControlCenter(void) {
  pitchPID = new PID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, AUTOMATIC);
  rollPID = new PID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, AUTOMATIC);
  altitudePID = new PID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, KP, KI, KD, AUTOMATIC);

  pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  altitudePID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void ControlCenter::computePIDs(void) {
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

void ControlCenter::setESCs(void) {
  escs->changePitch(pitchOutput);
  escs->changeRoll(rollOutput);
  escs->changeAltitude(altitudeOutput);
}
