#include <Arduino.h>
#include <math.h>
#include "dcm.h"

void DCM::begin(void) {
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;
  now = 0;
}

void DCM::update(void) {
  now = micros();
  sampleFreq = 1.0 / ((now - lastUpdate) / 1000000.0);
  lastUpdate = now;
  updateQuaternions();
  updateEulerAngles();
}

void DCM::updateQuaternions(void) {
  float recipNorm;
  float halfex = 0.0f, halfey = 0.0f, halfez = 0.0f;
  float qa, qb, qc;
  float halfvx, halfvy, halfvz;
  float invSampleFreq = (1.0f / sampleFreq);
  float halfInvSampleFreq = 0.5f * invSampleFreq;

  float ax = acc->x;
  float ay = acc->y;
  float az = acc->z;

  float gx = gyro->x;
  float gy = gyro->y;
  float gz = gyro->z;

  // Normalise accelerometer measurement
  recipNorm = invSqrt(ax * ax + ay * ay + az * az);
  ax *= recipNorm;
  ay *= recipNorm;
  az *= recipNorm;
  
  // Estimated direction of gravity
  halfvx = q1*q3 - q0*q2;
  halfvy = q0*q1 + q2*q3;
  halfvz = q0*q0 - 0.5f + q3*q3;

  // Error is sum of cross product between estimated direction and measured direction of field vectors
  halfex += (ay * halfvz - az * halfvy);
  halfey += (az * halfvx - ax * halfvz);
  halfez += (ax * halfvy - ay * halfvx);

  if(halfex != 0.0f && halfey != 0.0f && halfez != 0.0f) {
    // Compute and apply integral feedback if enabled
    integralFBx += DCM_TWO_KI * halfex * invSampleFreq;  // integral error scaled by Ki
    integralFBy += DCM_TWO_KI * halfey * invSampleFreq;
    integralFBz += DCM_TWO_KI * halfez * invSampleFreq;
    gx += integralFBx;  // apply integral feedback
    gy += integralFBy;
    gz += integralFBz;
  }

  // Apply proportional feedback
  gx += DCM_TWO_KP * halfex;
  gy += DCM_TWO_KP * halfey;
  gz += DCM_TWO_KP * halfez;
  
  // Integrate rate of change of quaternion
  gx *= (halfInvSampleFreq);   // pre-multiply common factors
  gy *= (halfInvSampleFreq);
  gz *= (halfInvSampleFreq);
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += ( qa * gx + qc * gz - q3 * gy);
  q2 += ( qa * gy - qb * gz + q3 * gx);
  q3 += ( qa * gz + qb * gy - qc * gx);
  
  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}

void DCM::updateEulerAngles(void) {
  x = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  y = asin(2 * (q0*q2 - q1*q3));
  z = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

float DCM::pitch(void) {
  return x;
}

float DCM::roll(void) {
  return y;
}

void DCM::print(void) {
  Serial.print("Quaternions: ");
  Serial.print("q0: "); Serial.print(q0, DEC);
  Serial.print(", q1: "); Serial.print(q1, DEC);
  Serial.print(", q2: "); Serial.print(q2, DEC);
  Serial.print(", q3: "); Serial.print(q3, DEC);
  Serial.print("Euler angles: ");
  Serial.print("x: "); Serial.print(x, DEC);
  Serial.print("y: "); Serial.print(y, DEC);
  Serial.print("z: "); Serial.print(z, DEC);
  Serial.println("");
}

void DCM::printForGraph(void) {
  Serial.print(q0, DEC); Serial.print('\t');
  Serial.print(q1, DEC); Serial.print('\t');
  Serial.print(q2, DEC); Serial.print('\t');
  Serial.print(q3, DEC); Serial.print('\t');
  Serial.print(x, DEC); Serial.print('\t');
  Serial.print(y, DEC); Serial.print('\t');
  Serial.print(z, DEC);
}

float DCM::invSqrt(float number) {
  volatile long i;
  volatile float x, y;
  volatile const float f = 1.5F;

  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f375a86 - ( i >> 1 );
  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  return y;
}
