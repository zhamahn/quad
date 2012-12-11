#ifndef dcm_h
#define dcm_h

#include "itg3200.h"
#include "adxl345.h"

class DCM {
  public:
    float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
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
    float Kp; // Proportional gin governs rate of convergence to accelerometer/magnetometer
    float Ki; // Integral gain governs rate of convergence of gyroscope biases
    float exInt, // scaled integral error
          eyInt,
          ezInt;
    float previousEx,
          previousEy,
          previousEz;

    bool isSwitched(float, float);
    long int lastUpdate;
    void updateQuaternions(void);
    void updateEulerAngles(void);
};

#endif
