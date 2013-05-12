#include "control_center.h"
#include "ahrs.h"
#include "controller.h"
#include "esc.h"
#include "quat.h"

ControlCenter::ControlCenter(ESC *_escs[], int _escs_count) {
  escs = _escs;
  escs_count = _escs_count;
}

void ControlCenter::update(void) {
  updateTargetQuat();
  updateErrorQuat();
  updateOutputs();
  setOutputs();
}

void ControlCenter::setOutputs(void) {
  int i;

  for (i=0; i<escs_count; i++) {
    escs[i]->changePitch(   pitchOutput);
    escs[i]->changeRoll(    rollOutput);
    escs[i]->changeYaw(     yawOutput);
    escs[i]->changeAltitude(altitudeOutput);
  }

  for (i=0; i<escs_count; i++) {
    escs[i]->write();
  }
}

void ControlCenter::updateTargetQuat(void) {
  float pitch = CONTROLLER_AXIS_TO_RAD(controller->stick_left_y);
  float roll  = CONTROLLER_AXIS_TO_RAD(controller->stick_left_x);
  float yaw   = ahrs->yaw() + CONTROLLER_AXIS_TO_RAD(controller->stick_right_x);

  float sin_pitch = sin(pitch/2);
  float cos_pitch = cos(pitch/2);
  float sin_roll  = sin(roll/2);
  float cos_roll  = cos(roll/2);
  float sin_yaw   = sin(yaw/2);
  float cos_yaw   = cos(yaw/2);

  target_quat.w = cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw;
  target_quat.i = sin_roll*cos_pitch*cos_yaw - cos_roll*sin_pitch*sin_yaw;
  target_quat.j = cos_roll*sin_pitch*cos_yaw + sin_roll*cos_pitch*sin_yaw;
  target_quat.k = cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw;
}

void ControlCenter::updateErrorQuat(void) {
  Quat inverted_quat(ahrs->quat);
  inverted_quat.invert();

  q_product(&error_quat, &inverted_quat, &target_quat);
  error_quat.normalize();
}

// positive value is upward
int ControlCenter::altitudeError(void) {
  int currentZAcc = ahrs->globAccZ() + ADXL_1G; // Should be ~0 in stable hover
  return currentZAcc + controller->trigger_right - controller->trigger_left;
}

float ControlCenter::pd(float error, float rate, float Kp, float Kd) {
  return (Kp * error) + (Kd * rate);
}

void ControlCenter::updateOutputs(void) {
  pitchOutput    = PITCH_OUTPUT_SCALE    * pd(error_quat.rotY(), gyro->y, PITCH_KP, PITCH_KD);
  rollOutput     = ROLL_OUTPUT_SCALE     * pd(error_quat.rotX(), gyro->x, ROLL_KP, ROLL_KD);
  yawOutput      = YAW_OUTPUT_SCALE      * pd(error_quat.rotZ(), gyro->z, YAW_KP, YAW_KD);
  altitudeOutput = ALTITUDE_OUTPUT_SCALE * altitudeError();
}
