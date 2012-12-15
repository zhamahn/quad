#ifndef dcm_h
#define dcm_h

#include "itg3200.h"
#include "adxl345.h"

#define DCM_TWO_KP  (3.0f) // original value: 1.0
#define DCM_TWO_KI  (0.1f) // original value: 0.2

//#define DEGREES(rad) ((rad) * 57.2957795130)
#define DEGREES(rad) ((rad) * 180 / M_PI)

class DCM {
  public:
    float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
    int pitch, roll, yaw; // Estimated euler angles

    ITG3200 *gyro;
    ADXL345 *acc;

    void begin(void);
    void update(void);
#ifdef DEBUG
    void print(void);
    void printForGraph(void);
#endif

  private:
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;     // scaled integral error
    float integralFBx, integralFBy, integralFBz;
    unsigned long lastUpdate, now; // sample period in milliseconds
    float sampleFreq; // half the sample period in seconds
    int startLoopTime;

    void updateQuaternions(void);
    void updateEulerAngles(void);
    float invSqrt(float);
};

#endif
