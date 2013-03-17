#include <PID_v1.h>

#include "control_center.h"
#include "ahrs.h"
#include "controller.h"
#include "esc.h"

ControlCenter::ControlCenter(ESC *_escs[], int _escs_count) {
  escs = _escs;
  escs_count = _escs_count;
  pitchPID    = new PID(&pitchInput,    &pitchOutput,    &pitchSetpoint,    PITCH_KP, PITCH_KI, PITCH_KD, AUTOMATIC);
  rollPID     = new PID(&rollInput,     &rollOutput,     &rollSetpoint,     ROLL_KP,  ROLL_KI,  ROLL_KD,  AUTOMATIC);
  yawPID      = new PID(&yawInput,      &yawOutput,      &yawSetpoint,      YAW_KP,   YAW_KI,   YAW_KD,   AUTOMATIC);
  altitudePID = new PID(&altitudeInput, &altitudeOutput, &altitudeSetpoint, ALT_KP,   ALT_KI,   ALT_KD,   AUTOMATIC);

  //pitchPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //rollPID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
  //altitudePID->SetOutputLimits(OUTPUT_MIN, OUTPUT_MAX);
}

void ControlCenter::update(void) {
  updateDesiredQuaternions();
  updateErrorQuaternions();
  updateErrorEulerAngles();
  updateOutputs();
  setOutputs();
}

void ControlCenter::updatePIDs(void) {
  pitchInput    = gyro->x;
  rollInput     = gyro->y;
  yawInput      = gyro->z;
  altitudeInput = 0;

  pitchSetpoint    = pitchError;
  rollSetpoint     = rollError;
  yawSetpoint      = yawError;
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

void ControlCenter::updateDesiredQuaternions(void) {
  float radPitch = RADS(map(controller->left_stick_y, -128, 127, -45, 45));
  float radRoll  = RADS(map(controller->left_stick_x, -128, 127, -45, 45));
  float radYaw   = ahrs->yaw() + RADS(controller->right_stick_x);

  float sinPitch = sin(radPitch/2);
  float cosPitch = cos(radPitch/2);
  float sinRoll  = sin(radRoll/2);
  float cosRoll  = cos(radRoll/2);
  float sinYaw   = sin(radYaw/2);
  float cosYaw   = cos(radYaw/2);

  desired_q0 = cosRoll*cosPitch*cosYaw + sinRoll*sinPitch*sinYaw;
  desired_q1 = sinRoll*cosPitch*cosYaw - cosRoll*sinPitch*sinYaw;
  desired_q2 = cosRoll*sinPitch*cosYaw + sinRoll*cosPitch*sinYaw;
  desired_q3 = cosRoll*cosPitch*sinYaw - sinRoll*sinPitch*cosYaw;
}

void ControlCenter::updateErrorQuaternions(void) {
  error_q0 = desired_q0/ahrs->q1 - desired_q1/ahrs->q1 - desired_q2/ahrs->q2 - desired_q3/ahrs->q3;
  error_q1 = desired_q0/ahrs->q0 + desired_q1/ahrs->q1 - desired_q2/ahrs->q3 + desired_q3/ahrs->q2;
  error_q2 = desired_q0/ahrs->q3 + desired_q1/ahrs->q1 + desired_q2/ahrs->q0 - desired_q3/ahrs->q1;
  error_q3 = desired_q0/ahrs->q2 - desired_q1/ahrs->q1 + desired_q2/ahrs->q1 + desired_q3/ahrs->q0;
}

void ControlCenter::updateErrorEulerAngles(void) {
  pitchError = -asin(2*error_q1*error_q3 + 2*error_q0*error_q2);
  rollError  = atan2(2*error_q2*error_q3 - 2*error_q0*error_q1, 2*error_q0*error_q0 + 2*error_q3*error_q3 - 1);
  yawError   = atan2(2*error_q1*error_q2 - 2*error_q0*error_q3, 2*error_q0*error_q0 + 2*error_q1*error_q1 - 1);
}

float ControlCenter::pd(float error, float rate, float Kp, float Kd) {
  return (Kp * error) + (Kd * rate);
}

void ControlCenter::updateOutputs(void) {
  pitchOutput = pd(pitchError, gyro->y, PITCH_KP, PITCH_KD);
  rollOutput  = pd(rollError,  gyro->x, ROLL_KP, ROLL_KD);
  yawOutput   = pd(yawError,   gyro->z, YAW_KP, YAW_KD);
}
