#ifndef control_center_h
#define control_center_h

#include <PID_v1.h>

#include "esc.h"
#include "controller.h"
#include "ahrs.h"

#define PITCH_KP 2
#define PITCH_KI 5
#define PITCH_KD 1

#define ROLL_KP 2
#define ROLL_KI 5
#define ROLL_KD 1

#define ALT_KP 2
#define ALT_KI 5
#define ALT_KD 1

#define YAW_KP 2
#define YAW_KI 5
#define YAW_KD 1

class ControlCenter {
  public:
    Controller *controller;
    AHRS *ahrs;

    // PID stuff
    PID *pitchPID;
    PID *rollPID;
    PID *altitudePID;
    PID *yawPID;

    ESC *escs[];
    int escs_count;

    double pitchInput, pitchOutput, pitchSetpoint;
    double rollInput, rollOutput, rollSetpoint;
    double altitudeInput, altitudeOutput, altitudeSetpoint;
    double yawInput, yawOutput, yawSetpoint;

    // functions
    ControlCenter(ESC *[], int);

    void updatePIDs(void);
    void setOutputs(void);

  private:
};

#endif
