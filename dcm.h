#ifndef dcm_h
#define dcm_h

#include "itg3200.h"
#include "adxl345.h"

#define DCM_KP 0.5f
#define DCM_KI 0.1f
#define DCM_TWO_KP  (2.0f * DCM_KP) // 2 * proportional gain
#define DCM_TWO_KI  (2.0f * DCM_KI) // 2 * integral gain

class DCM {
  public:
    volatile float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
    float x, y, z; // Estimated euler angles

    ITG3200 *gyro;
    ADXL345 *acc;

    void begin(void);
    void update(void);

    float pitch(void);
    float roll(void);
#ifdef DEBUG
    void print(void);
    void printForGraph(void);
#endif

  private:
    float iq0, iq1, iq2, iq3;
    float exInt, eyInt, ezInt;     // scaled integral error
    volatile float integralFBx, integralFBy, integralFBz;
    unsigned long lastUpdate, now; // sample period in milliseconds
    float sampleFreq; // half the sample period in seconds
    int startLoopTime;

    void updateQuaternions(void);
    void updateEulerAngles(void);
    float invSqrt(float);
};

#endif
