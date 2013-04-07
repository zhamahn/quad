#include <Wire.h>

#include "adxl345.h"
#include "utils.h"

void ADXL345::begin(void) {
  //Put the accelerometer in MEASURE mode
  writeReg(ADXL_ADDR, POWER_CTL, MEASURE);

  //Set the Range to +/- 4G
  writeReg(ADXL_ADDR, DATA_FORMAT, RANGE_0);
}

void ADXL345::read(void) {
  readReg(ADXL_ADDR, DATAX0, 6);

  if (Wire.available() >= 6) {
    rawX  = Wire.read();
    rawX |= Wire.read()<<8;
    rawY  = Wire.read();
    rawY |= Wire.read()<<8;
    rawZ  = Wire.read();
    rawZ |= Wire.read()<<8;
  }
}

void ADXL345::update(void) {
  read();
  // Compensate for sensor orientation here
  x = smooth(rawX, x, ADXL_SMOOTH_FACTOR);
  y = smooth(rawY*-1, y, ADXL_SMOOTH_FACTOR);
  z = smooth(rawZ*-1, z, ADXL_SMOOTH_FACTOR);
}

float ADXL345::valueToG(int value) {
  return (float)value * ADXL_SCALE_FACTOR;
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
  Serial.print(x, DEC); Serial.print('\t');
  Serial.print(y, DEC); Serial.print('\t');
  Serial.print(z, DEC);
}
#endif
