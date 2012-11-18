#ifndef helpers_h
#define helpers_h
#include <Arduino.h>

#define PI 3.14
#define PI_PER_180 0.01745
#define DEG_TO_RAD(deg) (deg * PI_PER_180 )

void writeReg(byte dev, byte reg, byte val);
void readReg(int dev, int reg, int count);
void debug(const char *msg);
#endif
