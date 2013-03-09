#ifndef utils_h
#define utils_h
#include <Arduino.h>

#define DEGREES(rads) ((rads) * 180 / M_PI)
#define RADS(degrees) ((degrees) * M_PI / 180)

void writeReg(byte dev, byte reg, byte val);
void readReg(int dev, int reg, int count);
int smooth(int, int, float);
//float invSqrt(float);
#endif
