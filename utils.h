#ifndef utils_h
#define utils_h
#include <Arduino.h>

void writeReg(byte dev, byte reg, byte val);
void readReg(int dev, int reg, int count);
void debug(const char *msg);
int smooth(int, int, float);
#endif