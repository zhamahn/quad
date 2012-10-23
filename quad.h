#ifndef quad_h
#define quad_h

#include "esc.h"
#include "itg3200.h"
#include "mma7660.h"
#include "ping.h"
#include "controller.h"
#include "escs.h"

class Quad {
  public:
    ESCs *escs;
    Controller *controller;
    ITG3200 *gyro;
    MMA7660 *acc;
    Ping *alt;

    void stabilize(void);
    void landNow(void);
    void setAltitude(int);

    void preFlight(void);

  private:
};

#endif
