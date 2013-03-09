#ifndef dcm_h
#define dcm_h

#include "itg3200.h"
#include "adxl345.h"
#include "hmc5883l.h"
#include "utils.h"

#define DCM_XAXIS 0
#define DCM_YAXIS 1
#define DCM_ZAXIS 2

#define DCM_ACC_KP 0.0
#define DCM_ACC_KI 0.0
#define DCM_MAG_KP 0.0
#define DCM_MAG_KI 0.0

class DCM {
  public:
    float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
    int pitch, roll, yaw; // Estimated euler angles
    float earthAccel[3];  // Accelerations in earth coordinates

    ITG3200 *gyro;
    ADXL345 *acc;
    HMC5883L *mag;

    void begin(void);
    void update(void);
    void earthAxisAccel(int);
#ifdef DEBUG
    void print(void);
    void printForGraph(void);
#endif

  private:
    float iq0, iq1, iq2, iq3;
    unsigned long lastUpdate; // sample period in milliseconds

    void updateQuaternions(void);
    void updateEulerAngles(void);
};

#endif
