#ifndef esc_h
#define esc_h
#define ESC_STEP 4

#include "quad.h"

class ESC {
  public:
    unsigned char pin;
    int gain;
    int output;

    ESC(unsigned char);

    int write(void);
    int set(int);
    int change(int);
};

#endif
