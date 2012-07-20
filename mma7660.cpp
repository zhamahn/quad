#include <Arduino.h>
#include <Wire.h>
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
  char val = 64;
  char data[3];
  char i;
  readReg(MMA7660addr, MMA7660_X, 3);

  if (Wire.available()) {
    for (i = 0; i < 3; i++) {
      while ( val > 63 ) // Values above 63 are invalid
        val = Wire.read();

      // transform the 7 bit signed number into an 8 bit signed number.
      data[i] = ((char)(val<<2));
      val = 64;
    }
  }

  dx = data[0] - x;
  dy = data[1] - y;
  dz = data[2] - z;
  x = data[0];
  y = data[1];
  z = data[2];
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
  return ( (x > MMA7660_STABILITY_THRESHOLD) || (x < MMA7660_STABILITY_THRESHOLD) );
}

bool MMA7660::stableY(void) {
  return ( (y > MMA7660_STABILITY_THRESHOLD) || (y < MMA7660_STABILITY_THRESHOLD) );
}
