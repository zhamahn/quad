#ifndef esc_h
#define esc_h
#include <PID_v1/PID_v1.h>
#define ESC_STEP 4

class ESC {
  public:

  double input, output, setpoint;
  int correction;
  PID * pid;
  int pin;

  ESC(int);

  double set(void);
  double set(double);
  double increase(void);
  double increase(int);
  double decrease(void);
  double decrease(int);
};

#endif
