#include <Arduino.h>
#include <Wire.h>

#include "adxl345.h"
#include "utils.h"

void ADXL345::begin(void) {
  //Put the accelerometer in MEASURE mode
  writeReg(ADXL_ADDR, POWER_CTL, MEASURE);

  //Set the Range to +/- 4G
  writeReg(ADXL_ADDR, DATA_FORMAT, RANGE_0);

  //default ADXL345 rate is 100 Hz. Perfect!
  smoothFactor = 0.1;
}

void ADXL345::read(void) {
  char i;
  readReg(ADXL_ADDR, DATAX0, 6);

  if (Wire.available()) {
    for (i = 0; i < 6; i++) {
      switch (i) {
        case 0: rawX  = Wire.read();    break;
        case 1: rawX |= Wire.read()<<8; break;
        case 2: rawY  = Wire.read();    break;
        case 3: rawY |= Wire.read()<<8; break;
        case 4: rawZ  = Wire.read();    break;
        case 5: rawZ |= Wire.read()<<8; break;
      }
    }
  }
}

void ADXL345::update(void) {
  read();
  x = smooth(rawX, x, smoothFactor);
  y = smooth(rawY, y, smoothFactor);
  z = smooth(rawZ, z, smoothFactor);
}
