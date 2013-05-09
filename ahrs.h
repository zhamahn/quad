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
#include "quat.h"

#define AHRS_XAXIS 0
#define AHRS_YAXIS 1
#define AHRS_ZAXIS 2

#define AHRS_ACC_KP 1.0 // reference 0.2
#define AHRS_ACC_KI 0.005 // reference 0.0005
#define AHRS_MAG_KP 2.0 // reference 2.0
#define AHRS_MAG_KI 0.005 // reference 0.005

class AHRS {
  public:
    Quat *quat;
    ITG3200 *gyro;
    ADXL345 *acc;
    HMC5883L *mag;

    void begin(void);
    void update(void);

    float pitch(void);
    float roll(void);
    float yaw(void);

    float globAccZ(void);
    float globAccY(void);
    float globAccX(void);
  private:
    unsigned long lastUpdate; // sample period in milliseconds

    float exInt, eyInt, ezInt; // scaled integral error

    void updateQuat(void);
};

#endif
