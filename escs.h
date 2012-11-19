#ifndef escs_h
#define escs_h

#include "esc.h"

#include "quad.h"

class ESCs {
  public:
    ESC *x;
    ESC *nx;
    ESC *y;
    ESC *ny;

    void write(void);
    void set(int);
    void setGains(void);

    void changePitch(int);
    void changeRoll(int);
    void changeAltitude(int);

  private:
    double average(void);
};
#endif
