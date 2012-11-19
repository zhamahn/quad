#include <Arduino.h>
#include <Wire.h>

#include "adxl345.h"
#include "utils.h"

void ADXL345::begin(void) {
  //Put the accelerometer in MEASURE mode
  writeReg(ADXL_ADDR, POWER_CTL, MEASURE);

  //Set the Range to +/- 4G
  writeReg(ADXL_ADDR, DATA_FORMAT, RANGE_0);

  smoothFactor = 0.4;
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
  smoothX = smooth(rawX, x, smoothFactor);
  smoothY = smooth(rawY, y, smoothFactor);
  smoothZ = smooth(rawZ, z, smoothFactor);
  x = valueToG(smoothX);
  y = valueToG(smoothY);
  z = valueToG(smoothZ);
}

float ADXL345::valueToG(int value) {
  // 4G == 0.0078
  return (float)value*0.0078;
}
