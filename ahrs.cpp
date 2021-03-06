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
#include "quat.h"

void AHRS::begin(void) {
  exInt = 0.0;
  eyInt = 0.0;
  ezInt = 0.0;
  lastUpdate = micros();
}

void AHRS::update(void) {
  updateQuat();
  lastUpdate = micros();
}

void AHRS::updateQuat(void) {
  float norm;
  float bx, by, bz;
  float vx, vy, vz;
  float wx, wy, wz;
  float exAcc, eyAcc, ezAcc;
  float exMag, eyMag, ezMag;

  float q0 = quat.w;
  float q1 = quat.i;
  float q2 = quat.j;
  float q3 = quat.k;

  float ax = acc->x;
  float ay = acc->y;
  float az = acc->z;

  float mx = mag->x;
  float my = mag->y;
  float mz = acc->z;

  float gx = gyro->x;
  float gy = gyro->y;
  float gz = gyro->z;

  float halfT = (micros() - lastUpdate) / 1000000.0 / 2;

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

  // Estimated direction of gravity
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;

  // Normalize magnetometer measurement
  norm = sqrt(mx*mx + my*my + mz*mz);
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;

  // compute reference direction of flux
  bx = mx * 2*(0.5 - q2q2 - q3q3) + my * 2*(q1q2 - q0q3)       + mz * 2*(q1q3 + q0q2);
  by = mx * 2*(q1q2 + q0q3)       + my * 2*(0.5 - q1q1 - q3q3) + mz * 2*(q2q3 - q0q1);
  bz = mx * 2*(q1q3 - q0q2)       + my * 2*(q2q3 + q0q1)       + mz * 2*(0.5 - q1q1 - q2q2);
  
  // Estimated direction of flux
  wx = bx * 2*(0.5 - q2q2 - q3q3) + by * 2*(q1q2 + q0q3)       + bz * 2*(q1q3 - q0q2);
  wy = bx * 2*(q1q2 - q0q3)       + by * 2*(0.5 - q1q1 - q3q3) + bz * 2*(q2q3 + q0q1);
  wz = bx * 2*(q1q3 + q0q2)       + by * 2*(q2q3 - q0q1)       + bz * 2*(0.5 - q1q1 - q2q2);

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

  // proportional error gain + integral error gain
  // adjusted gyroscope measurements
  gx += exAcc*AHRS_ACC_KP + exMag*AHRS_MAG_KP + exInt;
  gy += eyAcc*AHRS_ACC_KP + eyMag*AHRS_MAG_KP + eyInt;
  gz += ezAcc*AHRS_ACC_KP + ezMag*AHRS_MAG_KP + ezInt;

  // integrate quaternion rate
  quat.w += (-q1*gx - q2*gy - q3*gz) * halfT;
  quat.i += ( q0*gx + q2*gz - q3*gy) * halfT;
  quat.j += ( q0*gy - q1*gz + q3*gx) * halfT;
  quat.k += ( q0*gz + q1*gy - q2*gx) * halfT;

  quat.normalize();
}

float AHRS::pitch(void) {
  return quat.pitch();
}

float AHRS::roll(void) {
  return quat.roll();
}

float AHRS::yaw(void) {
  return quat.yaw();
}

float AHRS::globAccZ(void) {
  float accX;
  float accY;
  float accZ;

  // Estimated direction of gravity
  accX = 2*(quat.i*quat.k - quat.w*quat.j);
  accY = 2*(quat.w*quat.i + quat.j*quat.k);
  accZ = quat.w*quat.w - quat.i*quat.i - quat.j*quat.j + quat.k*quat.k;

  return accX*acc->x + accY*acc->y + accZ*acc->z;
}
