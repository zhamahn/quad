#ifndef control_center_h
#define control_center_h

#include <PID_v1.h>

#include "esc.h"
#include "itg3200.h"
#include "adxl345.h"
#include "ping.h"
#include "controller.h"
#include "escs.h"
#include "dcm.h"

#define ALTITUDE_OUTPUT_FACTOR 1
#define ROLL_OUTPUT_FACTOR 1
#define PITCH_OUTPUT_FACTOR 1
#define YAW_OUTPUT_FACTOR 1

#define ESC_X_CALIB 1
#define ESC_NX_CALIB 1
#define ESC_Y_CALIB 1
#define ESC_NY_CALIB 1

class ControlCenter {
  public:
    ESCs *escs;
    Controller *controller;
    ITG3200 *gyro;
    ADXL345 *acc;
    Ping *alt;
    DCM *dcm;

    // PID stuff
    PID *pitchPID;
    PID *rollPID;
    PID *altitudePID;
    PID *yawPID;

    double pitchInput, pitchOutput, pitchSetpoint;
    double rollInput, rollOutput, rollSetpoint;
    double altitudeInput, altitudeOutput, altitudeSetpoint;
    double yawInput, yawOutput, yawSetpoint;

    // functions
    void begin(void);

    void updatePIDs(void);
    void setOutputs(void);

  private:
};

#endif
