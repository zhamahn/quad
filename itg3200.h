#ifndef itg3200_h
#define itg3200_h
#include "main.h"
#define ITG3200addr   0x68
#define ITG3200_WHO 	0x00
#define ITG3200_SMPL	0x15
#define ITG3200_DLPF	0x16
#define ITG3200_INT_C	0x17
#define ITG3200_INT_S	0x1A
#define ITG3200_TEMP_H	0x1B
#define ITG3200_TEMP_L	0x1C
#define ITG3200_GX_H	0x1D
#define ITG3200_GX_L	0x1E
#define ITG3200_GY_H	0x1F
#define ITG3200_GY_L	0x20
#define ITG3200_GZ_H	0x21
#define ITG3200_GZ_L	0x22
#define ITG3200_PWR_M	0x3E
class ITG3200 {
  public:
  int x;
  int y;
  int z;

  void init(void);
  void read(void);
  void print(void);

  bool stable(void);
  bool stableY(void);
  bool stableX(void);
  bool stableZ(void);
};
#endif
