#ifndef mma7660_h
#define mma7660_h

#include "helpers.h"
#include "main.h"
#include "itg3200.h"

#define MMA7660addr   0x4c
#define MMA7660_X     0x00
#define MMA7660_Y     0x01
#define MMA7660_Z     0x02
#define MMA7660_TILT  0x03
#define MMA7660_SRST  0x04
#define MMA7660_SPCNT 0x05
#define MMA7660_INTSU 0x06
#define MMA7660_MODE  0x07
#define MMA7660_SR    0x08
#define MMA7660_PDET  0x09
#define MMA7660_PD    0x0A

class MMA7660 {
  public:
    char x, y, z;
    float pitch, roll;
    ITG3200 *gyro;

    void init(void);
    void read(void);
    void print(void);

  private:
    char x0, y0, z0;

    int kalman(int, int, int);
};
#endif
