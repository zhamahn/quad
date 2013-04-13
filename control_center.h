#ifndef control_center_h
#define control_center_h

#include <PID_v1.h>

#include "esc.h"
#include "controller.h"
#include "ahrs.h"
#include "adxl345.h"
#include "itg3200.h"
#include "hmc5883l.h"

#define PITCH_KP 2
#define PITCH_KI 5
#define PITCH_KD 0.5

#define ROLL_KP 2
#define ROLL_KI 5
#define ROLL_KD 0.5

#define ALT_KP 4
#define ALT_KI 5
#define ALT_KD 0.5

#define YAW_KP 3
#define YAW_KI 5
#define YAW_KD 0.5

class ControlCenter {
  public:
    Controller *controller;
    AHRS *ahrs;

    ITG3200 *gyro;
    ADXL345 *acc;
    HMC5883L *mag;

    // PID stuff
    PID *pitchPID;
    PID *rollPID;
    PID *altitudePID;
    PID *yawPID;

    ESC **escs;
    int escs_count;

    // functions
    ControlCenter(ESC *[], int);

    void updatePIDs(void);
    void update(void);

  private:
    float desired_q0;
    float desired_q1;
    float desired_q2;
    float desired_q3;

    float error_q0;
    float error_q1;
    float error_q2;
    float error_q3;

    float pitchError, rollError, yawError;

    double pitchInput, pitchOutput, pitchSetpoint;
    double rollInput, rollOutput, rollSetpoint;
    double altitudeInput, altitudeOutput, altitudeSetpoint;
    double yawInput, yawOutput, yawSetpoint;

    void updateTargetQuaternions(void);
    void updateErrorQuaternions(void);
    void updateErrorEulerAngles(void);
    void updateOutputs();
    void setOutputs(void);

    float pd(float, float, float, float);

};

#endif
