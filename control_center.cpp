#include <Arduino.h>
#include <PID_v1.h>

#include "control_center.h"
#include "quad.h"

void ControlCenter::begin(void) {
  pitchPID = new PID(&pitchInput, &pitchOutput, &pitchSetpoint, KP, KI, KD, AUTOMATIC);
  rollPID = new PID(&rollInput, &rollOutput, &rollSetpoint, KP, KI, KD, AUTOMATIC);
  altitudePID = new PID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, KP, KI, KD, AUTOMATIC);

  pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  altitudePID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void ControlCenter::updatePIDs(void) {
  pitchInput = dcm->pitch;
  rollInput = dcm->roll;
  //yawInput = dcm->yaw;
  //altitudeInput = (double)(alt->distance);

  pitchSetpoint = controller->pitch();
  rollSetpoint = controller->roll();
  //yawSetpoint = controller->yaw();
  //altitudeSetpoint = (double)(controller->altitude(alt->distance));

  pitchPID->Compute();
  rollPID->Compute();
  //yawPID->Compute();
  //altitudePID->Compute();
}

void ControlCenter::setOutputs(void) {
  double average = escs.average();
  escs->y->write(average + pitchOutput);
  escs->ny->write(average - pitchOutput);
  escs->x->write(average + rollOutput);
  escs->nx->write(average - rollOutput);
}
