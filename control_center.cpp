#include "control_center.h"
#include "ahrs.h"
#include "controller.h"
#include "esc.h"
#include "quat.h"

ControlCenter::ControlCenter(ESC *_escs[], int _escs_count) {
  escs = _escs;
  escs_count = _escs_count;
  error_quat = new Quat;
  target_quat = new Quat;
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
  float pitch = -1 * controllerAxis(controller->left_stick_y);
  float roll  = controllerAxis(controller->left_stick_x);
  float yaw   = ahrs->yaw() + controllerAxis(controller->right_stick_x);

  float sin_pitch = sin(pitch/2);
  float cos_pitch = cos(pitch/2);
  float sin_roll  = sin(roll/2);
  float cos_roll  = cos(roll/2);
  float sin_yaw   = sin(yaw/2);
  float cos_yaw   = cos(yaw/2);

  target_quat->w = cos_roll*cos_pitch*cos_yaw + sin_roll*sin_pitch*sin_yaw;
  target_quat->i = sin_roll*cos_pitch*cos_yaw - cos_roll*sin_pitch*sin_yaw;
  target_quat->j = cos_roll*sin_pitch*cos_yaw + sin_roll*cos_pitch*sin_yaw;
  target_quat->k = cos_roll*cos_pitch*sin_yaw - sin_roll*sin_pitch*cos_yaw;
}

void ControlCenter::updateErrorQuat(void) {
  Quat inverted_quat(ahrs->quat);
  inverted_quat.invert();

  q_product(error_quat, &inverted_quat, target_quat);
  error_quat->normalize();
}

//void ControlCenter::updateErrorEulerAngles(void) {
  //pitchError = -asin(2 * (error_q0*error_q2 - error_q1*error_q3));
  //rollError  = -atan2(2 * (error_q0*error_q1 + error_q2*error_q3), 1 - 2 * (error_q1*error_q1 + error_q2*error_q2));
  //yawError   = -atan2(2 * (error_q0*error_q3 + error_q1*error_q2), 1 - 2 * (error_q2*error_q2 + error_q3*error_q3));
//}

float ControlCenter::pd(float error, float rate, float Kp, float Kd) {
  return (Kp * error) + (Kd * rate);
}

void ControlCenter::updateOutputs(void) {
  pitchOutput    = PITCH_OUTPUT_SCALE      * pd(error_quat->pitch(), gyro->y, PITCH_KP, PITCH_KD);
  rollOutput     = ROLL_OUTPUT_SCALE       * pd(error_quat->roll(),  gyro->x, ROLL_KP, ROLL_KD);
  yawOutput      = YAW_OUTPUT_SCALE        * pd(error_quat->yaw(),   gyro->z, YAW_KP, YAW_KD);
  altitudeOutput = ALTITUDE_OUTPUT_SCALE   * (controller->right_trigger - controller->left_trigger);
}

float ControlCenter::controllerAxis(int value) {
  return RADS( map(value, STICK_AXIS_MIN, STICK_AXIS_MAX, -45, 45) );
}
