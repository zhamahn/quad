/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/
// code in this library is largery based on Kinematics_MARG found in AeroQuad

#include "ahrs.h"

void AHRS::begin(void) {
  q0 = 1.0;
  q1 = 0.0;
  q2 = 0.0;
  q3 = 0.0;
  lastUpdate = micros();
}

void AHRS::update(void) {
  updateQuaternions();
  //updateEulerAngles();
  //updateEarthAccels();
  lastUpdate = micros();
}

void AHRS::updateQuaternions(void) {
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
  exInt += exAcc*AHRS_ACC_KI + exMag*AHRS_MAG_KI;
  eyInt += eyAcc*AHRS_ACC_KI + eyMag*AHRS_MAG_KI;
  ezInt += ezAcc*AHRS_ACC_KI + ezMag*AHRS_MAG_KI;

  // adjusted gyroscope measurements
  gx += exAcc*AHRS_ACC_KP + exMag*AHRS_MAG_KP + exInt;
  gy += eyAcc*AHRS_ACC_KP + eyMag*AHRS_MAG_KP + eyInt;
  gz += ezAcc*AHRS_ACC_KP + ezMag*AHRS_MAG_KP + ezInt;

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

float AHRS::pitch(void) {
  return -asin(2*q1*q3 + 2*q0*q2);
}

float AHRS::roll(void) {
  return atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1);
}

float AHRS::yaw(void) {
  return atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1);
}

//void AHRS::updateEulerAngles(void) {
  //pitch = DEGREES(-asin(2*q1*q3 + 2*q0*q2)); // theta
  //roll  = DEGREES(atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1)); // phi
  //yaw   = DEGREES(atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1)); // psi
//}

//void AHRS::updateEarthAccels(void) {
  //int sin_pitch = sin(pitch);
  //int sin_roll = sin(roll);
  //earthAccel[AHRS_XAXIS] = acc->x / sin_pitch / sin_roll;
  //earthAccel[AHRS_YAXIS] = acc->y / sin_pitch / sin_roll;
  //earthAccel[AHRS_ZAXIS] = acc->z / sin_pitch / sin_roll;
//}

#ifdef DEBUG
void AHRS::print(void) {
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

void AHRS::printForGraph(void) {
  Serial.print(q0, DEC); Serial.print('\t');
  Serial.print(q1, DEC); Serial.print('\t');
  Serial.print(q2, DEC); Serial.print('\t');
  Serial.print(q3, DEC); Serial.print('\t');
  Serial.print(pitch, DEC); Serial.print('\t');
  Serial.print(roll, DEC); Serial.print('\t');
  Serial.print(yaw, DEC);
}
#endif
