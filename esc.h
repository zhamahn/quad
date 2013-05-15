#ifndef esc_h
#define esc_h

#include <Servo.h>

#define ESC_MIN 544
#define ESC_MAX 2400
#define ESC_OFF 520

#define PITCH_SCALE 10
#define ROLL_SCALE 10
#define YAW_SCALE 7
#define ALTITUDE_SCALE 0.01

class ESC {
  public:
    unsigned char pin;
    int output;

    float pitch_factor;
    float roll_factor;
    float yaw_factor;
    float altitude_factor;
    bool armed;

    ESC(unsigned char, float, float, float, float);
    void begin(void);

    bool arm(void);
    bool unarm(void);

    int write(void);
    int change(int);
    int changePitch(int);
    int changeRoll(int);
    int changeYaw(int);
    int changeAltitude(int);
  private:
    Servo servo;
};

#endif
