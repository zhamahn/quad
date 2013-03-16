#include <PID_v1.h>

#include "control_center.h"
#include "ahrs.h"
#include "controller.h"
#include "esc.h"

ControlCenter::ControlCenter(ESC *escs[], int _escs_count) {
  escs = escs;
  escs_count = _escs_count;
  pitchPID    = new PID(&pitchInput,    &pitchOutput,    &pitchSetpoint,    PITCH_KP, PITCH_KI, PITCH_KD, AUTOMATIC);
  rollPID     = new PID(&rollInput,     &rollOutput,     &rollSetpoint,     ROLL_KP,  ROLL_KI,  ROLL_KD,  AUTOMATIC);
  yawPID      = new PID(&yawInput,      &yawOutput,      &yawSetpoint,      YAW_KP,   YAW_KI,   YAW_KD,   AUTOMATIC);
  altitudePID = new PID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, ALT_KP,   ALT_KI,   ALT_KD,   AUTOMATIC);

  //pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //altitudePID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void ControlCenter::updatePIDs(void) {
  pitchInput    = ahrs->pitch;
  rollInput     = ahrs->roll;
  yawInput      = ahrs->yaw;
  altitudeInput = ahrs->earthAccel[AHRS_ZAXIS];

  pitchSetpoint    = map(controller->left_stick_y, -128, 127, -45, 45);
  rollSetpoint     = map(controller->left_stick_x, -128, 127, -45, 45);
  yawSetpoint      = controller->right_stick_x;
  altitudeSetpoint = controller->right_trigger - controller->left_trigger;

  pitchPID->Compute();
  rollPID->Compute();
  yawPID->Compute();
  altitudePID->Compute();
}

void ControlCenter::setOutputs(void) {
  int i;

  for (i=0; i<escs_count; i++) {
    escs[i]->changePitch(pitchOutput);
    escs[i]->changeRoll(rollOutput);
    escs[i]->changeYaw(yawOutput);
    escs[i]->changeAltitude(altitudeOutput);
  }

  for (i=0; i<escs_count; i++) {
    escs[i]->write();
  }
}
