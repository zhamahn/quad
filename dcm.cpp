#include <Arduino.h>
#include <math.h>
#include "dcm.h"

void DCM::begin(void) {
  exInt = eyInt = ezInt = 0.0;
  previousEx = previousEy = previousEz = 0.0;
  q0 = 1.0;
  q1 = q2 = q3 = 0.0;
  exInt = eyInt = ezInt = 0.0;
  Kp = 0.2;
  Ki = 0.0005;
  lastUpdate = micros();
}

void DCM::updateQuaternions(void) {
  long int halfSampleTime = (micros() - lastUpdate)/2;

  float norm;
  float vx, vy, vz;
  float ex, ey, ez;
  float ax, ay, az;
  float gx, gy, gz;
  
  // Normalize accelerometr values
  norm = sqrt( acc->x*acc->x + acc->y*acc->y + acc->z*acc->z );
  ax = acc->x / norm;
  ay = acc->y / norm;
  az = acc->z / norm;

  // Estimated direction of gravity and flux (v and w)
  vx = 2*(q1*q3 - q0*q2);
  vy = 2*(q0*q1 + q2*q3);
  vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  // Error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (vy*az - vz*ay);
  ey = (vz*ax - vx*az);
  ez = (vx*ay - vy*ax);

  // integral error scaled integral gain
  exInt = exInt + ex*Ki;
  if (isSwitched(previousEx,ex)) {
    exInt = 0.0;
  }
  previousEx = ex;
	
  eyInt = eyInt + ey*Ki;
  if (isSwitched(previousEy,ey)) {
    eyInt = 0.0;
  }
  previousEy = ey;

  ezInt = ezInt + ez*Ki;
  if (isSwitched(previousEz,ez)) {
    ezInt = 0.0;
  }
  previousEz = ez;
	
  // adjusted gyroscope measurements
  gx = gyro->x + Kp*ex + exInt;
  gy = gyro->y + Kp*ey + eyInt;
  gz = gyro->z + Kp*ez + ezInt;
    
  // integrate quaternion rate and normalise
  q0i = (-q1*gx - q2*gy - q3*gz) * halfSampleTime;
  q1i = ( q0*gx + q2*gz - q3*gy) * halfSampleTime;
  q2i = ( q0*gy - q1*gz + q3*gx) * halfSampleTime;
  q3i = ( q0*gz + q1*gy - q2*gx) * halfSampleTime;
  q0 += q0i;
  q1 += q1i;
  q2 += q2i;
  q3 += q3i;
    
  // normalise quaternion
  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  lastUpdate = micros();
}

void DCM::updateEulerAngles(void) {
  x = atan2(2 * (q0*q1 + q2*q3), 1 - 2 *(q1*q1 + q2*q2));
  y = asin(2 * (q0*q2 - q1*q3));
  z = atan2(2 * (q0*q3 + q1*q2), 1 - 2 *(q2*q2 + q3*q3));
}

bool DCM::isSwitched(float previousError, float currentError) {
  if ( (previousError > 0 && currentError < 0) ||
       (previousError < 0 && currentError > 0) ) {
    return true;
  }
  return false;
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
  Serial.print(q0, DEC); Serial.print(";");
  Serial.print(q1, DEC); Serial.print(";");
  Serial.print(q2, DEC); Serial.print(";");
  Serial.print(q3, DEC); Serial.print(";");
  Serial.print(x, DEC); Serial.print(";");
  Serial.print(y, DEC); Serial.print(";");
  Serial.print(z, DEC);
}
