#ifndef controller_h
#define controller_h
#include <HardwareSerial.h>

#define TRIGGER_ZERO 150
#define STICK_ZERO 128

struct Controller {
  signed char left_stick_x;
  signed char left_stick_y;

  signed char right_stick_x;
  signed char right_stick_y;

  unsigned char right_trigger;
  unsigned char left_trigger;

  int white;
  int black;

  bool up;
  bool down;
  bool left;
  bool right;

  bool left_stick;
  bool right_stick;
  bool back;
  bool start;
  bool a;
  bool b;
  bool x;
  bool y;

  unsigned long lastUpdateAt;

  void updateButtons(unsigned char);
  void updateDpad(unsigned char);
  void updateFromDataArray(unsigned char *);
  #ifdef DEBUG
  void print(HardwareSerial *);
  #endif

  signed char roll(void);
  signed char pitch(void);
  int altitude(int);

  signed char yawSpeed(void);

  void resetIfOldData(void);
  void reset(void);
};

#endif
