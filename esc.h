#ifndef esc_h
#define esc_h

#define OUTPUT_MIN 10
#define OUTPUT_MAX 255

class ESC {
  public:
    unsigned char pin;
    int output;

    float pitch_factor;
    float roll_factor;
    float yaw_factor;
    float altitude_factor;

    ESC(unsigned char, float, float, float, float);

    int write(void);
    int change(int);
    int changePitch(int);
    int changeRoll(int);
    int changeYaw(int);
    int changeAltitude(int);
};

#endif
