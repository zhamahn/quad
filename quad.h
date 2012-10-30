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
    PID *altitudePID;

    double pitchInput, pitchOutput, pitchSetpoint;
    double rollInput, rollOutput, rollSetpoint;
    double altitudeInput, altitudeOutput, altitudeSetpoint;

    // functions
    Quad(void);

    void computePIDs(void);
    void setESCs(void);

  private:
};

#endif
