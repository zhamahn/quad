#ifndef controller_h
#define controller_h
#include <SoftwareSerial.h>

#define TRIGGER_ZERO 150
#define STICK_ZERO 128

// Communication protocol
#define BYTE_IS_FRAME_HEADER(byte) (byte == 0xFF)
#define BYTE_IS_FRAME_FOOTER(byte) (byte >= 0xCC)
#define PARITY_BIT(byte) (byte & 0x03)
#define FRAME_DATA_LENGTH 8

#define FRAME_INCOMPLETE 0
#define FRAME_COMPLETE 1
#define FRAME_DISCARDED 2

class Controller {
  public:
    SoftwareSerial *mySerial;

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

    void update();
    void reset(void);
    #ifdef DEBUG
    void print();
    #endif

  private:
    unsigned long lastUpdateAt;

    int countOnes(unsigned char);
    int readFrame(unsigned char *);
    void updateButtons(unsigned char);
    void updateDpad(unsigned char);
    void updateFromDataArray(unsigned char *);

};

#endif
