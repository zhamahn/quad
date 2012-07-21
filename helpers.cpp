#include "helpers.h"
#include <Wire/Wire.h>
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
