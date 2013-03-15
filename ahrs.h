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

#ifndef ahrs_h
#define ahrs_h

#include "itg3200.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "utils.h"

#define AHRS_XAXIS 0
#define AHRS_YAXIS 1
#define AHRS_ZAXIS 2

#define AHRS_ACC_KP 0.0
#define AHRS_ACC_KI 0.0
#define AHRS_MAG_KP 0.0
#define AHRS_MAG_KI 0.0

class AHRS {
  public:
    float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
    float pitch, roll, yaw; // Estimated euler angles
    float earthAccel[3];  // Accelerations in earth coordinates

    ITG3200 *gyro;
    ADXL345 *acc;
    HMC5883L *mag;

    void begin(void);
    void update(void);
#ifdef DEBUG
    void print(void);
    void printForGraph(void);
#endif

  private:
    float iq0, iq1, iq2, iq3;
    unsigned long lastUpdate; // sample period in milliseconds

    void updateQuaternions(void);
    void updateEulerAngles(void);
    void updateEarthAccels(void);
};

#endif
