#include <Arduino.h>
#include <math.h>
#include "dcm.h"

void DCM::begin(void) {
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  lastUpdate = micros();
}

void DCM::update(void) {
  updateQuaternions();
  updateEulerAngles();
  lastUpdate = micros();
}

void DCM::updateQuaternions(void) {
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float q0i, q1i, q2i, q3i;
  float exAcc, eyAcc, ezAcc;
  float exMag, eyMag, ezMag;
  float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;

  float ax = acc->x;
  float ay = acc->y;
  float az = acc->z;

  float mx = mag->x;
  float my = mag->y;
  float mz = acc->z;

  float gx = gyro->x;
  float gy = gyro->y;
  float gz = gyro->z;

  float halfT = (micros() - lastUpdate) / 2;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  // Normalise accelerometer measurement
  norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  // Normalize magnetometer measurement
  norm = sqrt(mx*mx + my*my + mz*mz);
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;

  // compute reference direction of flux
  hx = mx * 2*(0.5 - q2q2 - q3q3) + my * 2*(q1q2 - q0q3)       + mz * 2*(q1q3 + q0q2);
  hy = mx * 2*(q1q2 + q0q3)       + my * 2*(0.5 - q1q1 - q3q3) + mz * 2*(q2q3 - q0q1);
  hz = mx * 2*(q1q3 - q0q2)       + my * 2*(q2q3 + q0q1)       + mz * 2*(0.5 - q1q1 - q2q2);

  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;
  
  // Estimated direction of gravity
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  // Estimated direction of flux
  wx = bx * 2*(0.5 - q2q2 - q3q3) + bz * 2*(q1q3 - q0q2);
  wy = bx * 2*(q1q2 - q0q3)       + bz * 2*(q0q1 + q2q3);
  wz = bx * 2*(q0q2 + q1q3)       + bz * 2*(0.5 - q1q1 - q2q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  exAcc = (vy*az - vz*ay);
  eyAcc = (vz*ax - vx*az);
  ezAcc = (vx*ay - vy*ax);
    
  exMag = (my*wz - mz*wy);
  eyMag = (mz*wx - mx*wz);
  ezMag = (mx*wy - my*wx);

  // integral error scaled integral gain
  exInt = exInt + exAcc*DCM_ACC_KI + exMag*DCM_MAG_KI;
  eyInt = eyInt + eyAcc*DCM_ACC_KI + eyMag*DCM_MAG_KI;
  ezInt = ezInt + ezAcc*DCM_ACC_KI + ezMag*DCM_MAG_KI;

  // adjusted gyroscope measurements
  gx = gx + exAcc*DCM_ACC_KP + exMag*DCM_MAG_KP + exInt;
  gy = gy + eyAcc*DCM_ACC_KP + eyMag*DCM_MAG_KP + eyInt;
  gz = gz + ezAcc*DCM_ACC_KP + ezMag*DCM_MAG_KP + ezInt;

  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfT;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfT;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfT;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfT;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;
  
  // Normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;
}

void DCM::updateEulerAngles(void) {
  pitch = (int)DEGREES( -asin(2*q1*q3 + 2*q0*q2) ); // theta
  roll  = (int)DEGREES( atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1) ); // phi
  yaw   = (int)DEGREES( atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) ); // psi
}

#ifdef DEBUG
void DCM::print(void) {
  Serial.print("Quaternions: ");
  Serial.print("q0: "); Serial.print(q0, DEC);
  Serial.print(", q1: "); Serial.print(q1, DEC);
  Serial.print(", q2: "); Serial.print(q2, DEC);
  Serial.print(", q3: "); Serial.print(q3, DEC);
  Serial.print("Euler angles: ");
  Serial.print("pitch: "); Serial.print(pitch, DEC);
  Serial.print("roll: "); Serial.print(roll, DEC);
  Serial.print("yaw: "); Serial.print(yaw, DEC);
  Serial.println("");
}

void DCM::printForGraph(void) {
  Serial.print(q0, DEC); Serial.print('\t');
  Serial.print(q1, DEC); Serial.print('\t');
  Serial.print(q2, DEC); Serial.print('\t');
  Serial.print(q3, DEC); Serial.print('\t');
  Serial.print(pitch, DEC); Serial.print('\t');
  Serial.print(roll, DEC); Serial.print('\t');
  Serial.print(yaw, DEC);
}
#endif
