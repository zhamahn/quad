#ifndef helpers_h
#define helpers_h
#include <Arduino.h>
void writeReg(byte dev, byte reg, byte val);
void readReg(int dev, int reg, int count);
void debug(const char *msg);
#endif
