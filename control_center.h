#ifndef control_center_h
#define control_center_h

#include <PID_v1.h>

#include "esc.h"
#include "controller.h"
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
    Controller *controller;
    DCM *dcm;

    // PID stuff
    PID *pitchPID;
    PID *rollPID;
    PID *altitudePID;
    PID *yawPID;

    // ESCS
    ESC *x;
    ESC *nx;
    ESC *y;
    ESC *ny;

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
