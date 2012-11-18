#include <Wire.h>

#include "utils.h"

void writeReg(byte dev, byte reg, byte val) {
  Wire.beginTransmission(dev);
  delay(100);
  Wire.write(reg);
  delay(10);
  Wire.write(val);
  delay(10);
  Wire.endTransmission();
}

void readReg(int dev, int reg, int count) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(dev, count);
}

void debug(const char *msg) {
  #ifdef DEBUG
    Serial.println(msg);
  #endif
}

int smooth(int rawData, int smoothedData, float factor) {
  if (factor > 1){      // check to make sure param's are within range
    factor = .99;
  }
  else if (factor <= 0){
    factor = 0;
  }
  return (int)( (rawData * (1 - factor)) + (smoothedData * factor) );
}
