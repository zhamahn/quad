#ifndef esc_h
#define esc_h
#include <PID_v1/PID_v1.h>
#define ESC_STEP 4

class ESC {
  public:
    int correction;
    int pin;
    int output;

    ESC(int);

    int write(void);
    int write(int);
    int increase(void);
    int increase(int);
    int decrease(void);
    int decrease(int);
    bool stopped(void);
    int change(int);
};

#endif
