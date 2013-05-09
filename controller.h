#ifndef controller_h
#define controller_h
#include <SoftwareSerial.h>

#define TRIGGER_ZERO 150
#define STICK_ZERO 128

// Communication protocol
#define FRAME_HEADER1 0xFF
#define FRAME_HEADER2 0xFF
#define FRAME_HEADER_COUNT 2
#define BYTE_IS_FRAME_FOOTER(byte) (byte >= 0xCC)
#define PARITY_BIT(byte) (byte & 0x03)
#define FRAME_DATA_LENGTH 8

#define FRAME_INCOMPLETE 0
#define FRAME_COMPLETE 1
#define FRAME_DISCARDED 2

#define STICK_AXIS_MAX 127
#define STICK_AXIS_MIN -128

#define CONTROLLER_DATA_ARRAY_LENGTH 14
#define CONTROLLER_AXIS_TO_RAD(value) (value * 0.002) // Limits to ~ +-45°

class Controller {
  public:
    SoftwareSerial *mySerial;

    int stick_left_x;
    int stick_left_y;

    int stick_right_x;
    int stick_right_y;

    int trigger_left;
    int trigger_right;

    int btn_white;
    int btn_black;

    bool btn_up;
    bool btn_down;
    bool btn_left;
    bool btn_right;

    bool btn_stick_left;
    bool btn_stick_right;
    bool btn_back;
    bool btn_start;
    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;

    void update();
    void reset(void);

  private:
    unsigned char data[CONTROLLER_DATA_ARRAY_LENGTH];
    unsigned long lastUpdateAt;

    int countOnes(unsigned char);
    int readFrame(void);
    void updateFromDataArray(void);
    void updateButtons(void);
};

#endif
