#include <Wire.h>

#include "hmc5883l.h"
#include "utils.h"

void HMC5883L::begin(void) {
  writeReg(HMC5883L_ADDR, HMC5883L_MODE, HMC5883L_MODE_CONT);
  writeReg(HMC5883L_ADDR, HMC5883L_CONFREG_B, HMC5883L_GN5); // Set measurement gain
}

void HMC5883L::read(void) {
  readReg(HMC5883L_ADDR, HMC5883L_X_MSB, 6);
  
  if (Wire.available() >= 6) {
    rawX  = Wire.read()<<8;
    rawX |= Wire.read();
    rawZ  = Wire.read()<<8;
    rawZ |= Wire.read();
    rawY  = Wire.read()<<8;
    rawY |= Wire.read();
  }
}

void HMC5883L::update(void) {
  read();
  // Compensate for sensor orientation here
  x = smooth(rawY, x, HMC5883L_SMOOTH_FACTOR);
  y = smooth(rawX, y, HMC5883L_SMOOTH_FACTOR);
  z = smooth(rawZ, z, HMC5883L_SMOOTH_FACTOR);
}
