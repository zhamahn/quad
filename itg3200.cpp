#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include "itg3200.h"
#include "utils.h"

void ITG3200::begin(void) {
  //Set internal clock to 1kHz with 42Hz LPF and Full Scale to 3 for proper operation
  writeReg(ITG_ADDR, DLPF_FS, DLPF_FS_SEL_0|DLPF_FS_SEL_1|DLPF_CFG_0);

  //Set sample rate divider for 100 Hz operation
  writeReg(ITG_ADDR, SMPLRT_DIV, 9);	//Fsample = Fint / (divider + 1) where Fint is 1kHz

  //Select X gyro PLL for clock source
  writeReg(ITG_ADDR, PWR_MGM, PWR_MGM_CLK_SEL_0);
}

void ITG3200::read(void) {
  char i;
  readReg(ITG_ADDR, GYRO_XOUT_H, 6);

  if (Wire.available()) {
    for (i = 0; i < 6; i++) {
      switch (i) {
        case 0: rawX  = Wire.read()<<8; break;
        case 1: rawX |= Wire.read();    break;
        case 2: rawY  = Wire.read()<<8; break;
        case 3: rawY |= Wire.read();    break;
        case 4: rawZ  = Wire.read()<<8; break;
        case 5: rawZ |= Wire.read();    break;
      }
    }
  }
}

void ITG3200::update(void) {
  read();
  // Apply error correction
  rawX += ITG_X_ERROR;
  rawY += ITG_Y_ERROR;
  rawZ += ITG_Z_ERROR;
  // Smoothen read values
  smoothX = smooth(rawX, smoothX, ITG_SMOOTH_FACTOR);
  smoothY = smooth(rawY, smoothY, ITG_SMOOTH_FACTOR);
  smoothZ = smooth(rawZ, smoothZ, ITG_SMOOTH_FACTOR);
  // Convert to radians
  x = (float)smoothX * ITG_SCALE_FACTOR * -1.0;
  y = (float)smoothY * ITG_SCALE_FACTOR * -1.0;
  z = (float)smoothZ * ITG_SCALE_FACTOR;
}

#ifdef DEBUG
void ITG3200::print(void) {
  Serial.print("Rotation: ");
  Serial.print("X: "); Serial.print(x, DEC);
  Serial.print(", Y: "); Serial.print(y, DEC);
  Serial.print(", Z: "); Serial.print(z, DEC);
  Serial.println("");
}
void ITG3200::printForGraph(void) {
  Serial.print(rawX, DEC); Serial.print('\t');
  Serial.print(rawY, DEC); Serial.print('\t');
  Serial.print(rawZ, DEC); Serial.print('\t');
  Serial.print(smoothX, DEC); Serial.print('\t');
  Serial.print(smoothY, DEC); Serial.print('\t');
  Serial.print(smoothZ, DEC); Serial.print('\t');
  Serial.print(x, DEC); Serial.print('\t');
  Serial.print(y, DEC); Serial.print('\t');
  Serial.print(z, DEC);
}
#endif
