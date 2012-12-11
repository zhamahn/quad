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
  x = smooth(rawX, x, smoothFactor);
  y = smooth(rawY, y, smoothFactor);
  z = smooth(rawZ, z, smoothFactor);
  Gx = valueToG(x);
  Gy = valueToG(y);
  Gz = valueToG(z);
}

float ADXL345::valueToG(int value) {
  // 4G == 0.0078
  return (float)value*0.0078;
}

#ifdef DEBUG
void ADXL345::print(void) {
  Serial.print("Acc: ");
  Serial.print("X: "); Serial.print(x, DEC);
  Serial.print(", Y: "); Serial.print(y, DEC);
  Serial.print(", Z: "); Serial.print(z, DEC);
  Serial.println("");
}
void ADXL345::printForGraph(void) {
  Serial.print(rawX, DEC); Serial.print('\t');
  Serial.print(rawY, DEC); Serial.print('\t');
  Serial.print(rawZ, DEC); Serial.print('\t');
  Serial.print(x, DEC); Serial.print('\t');
  Serial.print(y, DEC); Serial.print('\t');
  Serial.print(z, DEC); Serial.print('\t');
  Serial.print(Gx, DEC); Serial.print('\t');
  Serial.print(Gy, DEC); Serial.print('\t');
  Serial.print(Gz, DEC);
}
#endif
