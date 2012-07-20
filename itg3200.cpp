#include "itg3200.h"
#include <Arduino.h>
#include <Wire.h>
#include "helpers.h"
void ITG3200::init(void) {
  debug("Initializing gyroscope.");

  debug("--> reset.");
  writeReg(ITG3200addr, ITG3200_PWR_M, 0x80);
 
  debug("--> set sample rate divider.");
  writeReg(ITG3200addr, ITG3200_SMPL, 0x00);
 
  debug("--> set measurement accuracy.");
  writeReg(ITG3200addr, ITG3200_DLPF, 0x18);
}

void ITG3200::read(void) {
  char i;
  readReg(ITG3200addr, ITG3200_GX_L, 6);

  if (Wire.available()) {
    for (i = 0; i < 6; i++) {
      switch (i) {
        case 0: x  = Wire.read();    break;
        case 1: x |= Wire.read()<<8; break;
        case 2: y  = Wire.read();    break;
        case 3: y |= Wire.read()<<8; break;
        case 4: z  = Wire.read();    break;
        case 5: z |= Wire.read()<<8; break;
      }
    }
  }
}

void ITG3200::print(void) {
  Serial.print("Rotation: ");
  Serial.print("X: "); Serial.print(x, DEC);
  Serial.print(", Y: "); Serial.print(y, DEC);
  Serial.print(", Z: "); Serial.print(z, DEC);
  Serial.println("");
}

bool ITG3200::stable(void) {
  return (stableX() && stableY() && stableZ());
}

bool ITG3200::stableX(void) {
  return ( (x > ITG3200_STABILITY_THRESHOLD) || (x < ITG3200_STABILITY_THRESHOLD) );
}

bool ITG3200::stableY(void) {
  return ( (y > ITG3200_STABILITY_THRESHOLD) || (y < ITG3200_STABILITY_THRESHOLD) );
}

bool ITG3200::stableZ(void) {
  return ( (z > ITG3200_STABILITY_THRESHOLD) || (z < ITG3200_STABILITY_THRESHOLD) );
}
