#ifndef controller_h
#define controller_h
#include <HardwareSerial.h>

#define TRIGGER_ZERO 150
#define STICK_ZERO 128

struct Controller {
  unsigned char left_stick_x;
  unsigned char left_stick_y;
  unsigned char left_trigger;

  unsigned char right_stick_x;
  unsigned char right_stick_y;
  unsigned char right_trigger;

  unsigned char left_stick_x_delta;
  unsigned char left_stick_y_delta;
  unsigned char left_trigger_delta;

  unsigned char right_stick_x_delta;
  unsigned char right_stick_y_delta;
  unsigned char right_trigger_delta;

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
  void print(HardwareSerial *);

  unsigned char axisDelta(unsigned char *);
  void updateDeltas(void);
};

#endif
