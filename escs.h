#ifndef escs_h
#define escs_h

#include "esc.h"

struct ESCs {
  ESC *x;
  ESC *nx;
  ESC *y;
  ESC *ny;

  void computePIDs(void);
  void setOutputs(void);
  void decreaseOutputs(int = ESC_STEP);
  void increaseOutputs(int = ESC_STEP);
  void setCorrections(void);
  bool allStopped(void);
  double avgOutput(void);
};
#endif
