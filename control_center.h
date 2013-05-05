#ifndef control_center_h
#define control_center_h

#include "esc.h"
#include "controller.h"
#include "ahrs.h"
#include "adxl345.h"
#include "itg3200.h"
#include "hmc5883l.h"
#include "quat.h"

#define PITCH_KP 2.0
#define PITCH_KI 5.0
#define PITCH_KD 0.5

#define ROLL_KP 2.0
#define ROLL_KI 5.0
#define ROLL_KD 0.5

#define ALT_KP 4.0
#define ALT_KI 5.0
#define ALT_KD 0.5

#define YAW_KP 3.0
#define YAW_KI 5.0
#define YAW_KD 0.5

#define PITCH_OUTPUT_SCALE 1
#define ROLL_OUTPUT_SCALE 1
#define YAW_OUTPUT_SCALE 1
#define ALTITUDE_OUTPUT_SCALE 0.02

class ControlCenter {
  public:
    Controller *controller;
    AHRS *ahrs;

    ITG3200 *gyro;
    ADXL345 *acc;
    HMC5883L *mag;

    Quat *target_quat;
    Quat *error_quat;

    ESC **escs;
    int escs_count;

    // functions
    ControlCenter(ESC *[], int);

    void update(void);

  private:
    int pitchOutput;
    int rollOutput;
    int altitudeOutput;
    int yawOutput;

    void updateTargetQuat(void);
    void updateErrorQuat(void);
    void updateOutputs();
    void setOutputs(void);

    float controllerAxis(int);

    float pd(float, float, float, float);

};

#endif
