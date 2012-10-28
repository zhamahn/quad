#ifndef quad_h
#define quad_h

#include <PID_v1/PID_v1.h>

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

    // PID stuff
    PID *pitchPID;
    PID *rollPID;

    double pitchInput, pitchOutput, pitchSetpoint;
    double rollInput, rollOutput, rollSetpoint;

    // functions
    Quad(void);

    void stabilize(void);
    void landNow(void);
    void setAltitude(int);

    void preFlight(void);

    void computePIDs(void);

    void setESCs(void);

  private:
};

#endif
