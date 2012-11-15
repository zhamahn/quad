#ifndef dcm_h
#define dcm_h

class DCM {
  public:
    float q0, q1, q2, q3; // Quaternion elements representing the estimated orientation
    float x, y, z; // Estimated euler angles

    ITG3200 *gyro;
    ADXL345 *acc;

    void updateQuaternions(void);
    void updateEulerAngles(void);

  private:
    float Kp; // Proportional gin governs rate of convergence to accelerometer/magnetometer
    float Ki; // Integral gain governs rate of convergence of gyroscope biases
    float exInt, // scaled integral error
          eyInt,
          ezInt;
    float previousEx;
          previousEy;
          previousEz;
    bool isSwitched(float, float);
};

#endif
