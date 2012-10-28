#include <Arduino.h>
#include <Wire/Wire.h>
#include "mma7660.h"
#include "helpers.h"
// {{{ Functions
void MMA7660::init(void) {
  debug("Initializing accelerometer.");

  debug("--> Set to standby mode.");
  writeReg(MMA7660addr, MMA7660_MODE, 0x00);

  //debug("--> Set to generate automatic interrupt after every measurement");
  //writeReg(MMA7660addr, MMA7660_INTSU, 0x03);

  debug("--> Set sample rate.");
  writeReg(MMA7660addr, MMA7660_SR, 0x00);

  debug("--> Set pulse detection.");
  writeReg(MMA7660addr, MMA7660_PDET, 0x00);
  //writeReg(MMA7660addr, MMA7660_PDET, 0xE1);

  writeReg(MMA7660addr, MMA7660_PD, 0x04);
 
  debug("--> Set back to normal operation mode");
  writeReg(MMA7660addr, MMA7660_MODE, 0x01);
}

void MMA7660::read(void) {
  unsigned char val = 64;
  char data;
  char i;

  readReg(MMA7660addr, MMA7660_X, 3);

  if (Wire.available() > 0) {
    for (i = 0; i < 3; i++) {
      val = Wire.read();
      if (val > 64) // Values above 63 are invalid, continue to next loop
        continue;

      // transform the 7 bit signed number into an 8 bit signed number.
      // Bit 5 is the sign; move if left 2 bits so it becomes 7th bit (sign) of 8 bit number.
      // This multiplies the value by 4, so we need to divide it by 4
      data = ((char)(val<<2))/4;

      switch (i) {
        case 0: dx = data - x; x = data; break;
        case 1: dy = data - y; y = data; break;
        case 2: dz = data - z; z = data; break;
      };
    }
  }
}

void MMA7660::print(void) {
  Serial.print("Acceleration: ");
  Serial.print("X: "); Serial.print(x, DEC);
  //Serial.print(", DELTA_X: "); Serial.print(Acc.delta_x, DEC);
  Serial.print(", Y: "); Serial.print(y, DEC);
  //Serial.print(", DELTA_Y: "); Serial.print(Acc.delta_y, DEC);
  Serial.print(", Z: "); Serial.print(z, DEC);
  //Serial.print(", DELTA_z: "); Serial.print(Acc.delta_z, DEC);
  Serial.println("");
}

bool MMA7660::stable(void) {
  return (stableX() && stableY());
}

bool MMA7660::stableX(void) {
  return abs(x) > MMA7660_STABILITY_THRESHOLD;
}

bool MMA7660::stableY(void) {
  return abs(y) > MMA7660_STABILITY_THRESHOLD;
}

bool MMA7660::ascending(void) {
  return z > MMA7660_STABILITY_THRESHOLD;
}

bool MMA7660::descending(void) {
  return z < MMA7660_STABILITY_THRESHOLD;
}
