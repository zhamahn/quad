#include <Arduino.h>
#include <PID_v1.h>

#include "control_center.h"
#include "dcm.h"
#include "controller.h"
#include "esc.h"

void ControlCenter::begin(void) {
  pitchPID    = new PID(&pitchInput,    &pitchOutput,    &pitchSetpoint,    PITCH_KP, PITCH_KI, PITCH_KD, AUTOMATIC);
  rollPID     = new PID(&rollInput,     &rollOutput,     &rollSetpoint,     ROLL_KP,  ROLL_KI,  ROLL_KD,  AUTOMATIC);
  yawPID      = new PID(&yawInput,      &yawOutput,      &yawSetpoint,      YAW_KP,   YAW_KI,   YAW_KD,   AUTOMATIC);
  altitudePID = new PID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, ALT_KP,   ALT_KI,   ALT_KD,   AUTOMATIC);

  //pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //altitudePID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void ControlCenter::updatePIDs(void) {
  pitchInput = dcm->pitch;
  rollInput = dcm->roll;
  yawInput = dcm->yaw;
  altitudeInput = dcm->earthAccel[DCM_ZAXIS];

  pitchSetpoint = controller->pitch();
  rollSetpoint = controller->roll();
  yawSetpoint = controller->yaw();
  altitudeSetpoint = controller->altitude();

  pitchPID->Compute();
  rollPID->Compute();
  yawPID->Compute();
  altitudePID->Compute();
}

void ControlCenter::setOutputs(void) {
  double pitch = pitchOutput    * PITCH_OUTPUT_FACTOR;
  double roll  = rollOutput     * ROLL_OUTPUT_FACTOR;
  double alt   = altitudeOutput * ALTITUDE_OUTPUT_FACTOR;
  double yaw   = yawOutput      * YAW_OUTPUT_FACTOR;

  y->change(  ( pitch + alt - yaw) * ESC_Y_CALIB  );
  ny->change( (-pitch + alt + yaw) * ESC_NY_CALIB );
  x->change(  ( roll  + alt - yaw) * ESC_X_CALIB  );
  nx->change( (-roll  + alt + yaw) * ESC_NX_CALIB );
}
