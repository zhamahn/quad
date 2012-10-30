#ifndef escs_h
#define escs_h

#include "esc.h"

struct ESCs {
  ESC *x;
  ESC *nx;
  ESC *y;
  ESC *ny;

  void write(void);
  void write(int);
  void decrease(int = ESC_STEP);
  void increase(int = ESC_STEP);
  void setCorrections(void);
  bool allStopped(void);
  double avg(void);

  void changePitch(int);
  void changeRoll(int);
  void changeAltitude(int);
};
#endif
