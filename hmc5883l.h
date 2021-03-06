#ifndef hmc5883l
#define hmc5883l

#define HMC5883L_SMOOTH_FACTOR 0.1

class HMC5883L {
  public:
    int x, y, z;

    void begin(void);
    void update(void);

  private:
    int rawX, rawY, rawZ;
    void read(void);
};

#define HMC5883L_ADDR 0x1E // Or 0x3D
#define HMC5883L_MODE 0x02
#define HMC5883L_MODE_CONT 0x00
#define HMC5883L_MS_NORM 0x00
#define HMC5883L_MS_PBIAS 0x01
#define HMC5883L_MS_NBIAS 0x02
#define HMC5883L_CONFREG_A 0x00
#define HMC5883L_CONFREG_B 0x01
#define HMC5883L_X_MSB 0x03
#define HMC5883L_X_LSB 0x04
#define HMC5883L_Z_MSB 0x05
#define HMC5883L_Z_LSB 0x06
#define HMC5883L_Y_MSB 0x07
#define HMC5883L_Y_LSB 0x08
#define HMC5883L_STATUS 0x09

#define HMC5883L_GN0 (0x00<<5)
#define HMC5883L_GN1 (0x01<<5) // Default
#define HMC5883L_GN2 (0x02<<5)
#define HMC5883L_GN3 (0x03<<5)
#define HMC5883L_GN4 (0x04<<5)
#define HMC5883L_GN5 (0x05<<5)
#define HMC5883L_GN6 (0x06<<5)
#define HMC5883L_GN7 (0x07<<5)
#define HMC5883L_GN8 (0x08<<5)
#define HMC5883L_GN9 (0x09<<5)
#endif
