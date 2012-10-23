#ifndef main_h
#define main_h
#include <Arduino.h>
// {{{ Defines
// Pin definitions
#define LED 13
#define PING_PIN 7
#define PING_INT 0
#define PING_INT_PIN 2

#define ESC_X_PIN 10
#define ESC_NX_PIN 5
#define ESC_Y_PIN 8
#define ESC_NY_PIN 9

#define ESC_ALL -1
#define ESC_N 4

#define OUTPUT_MIN 10
#define OUTPUT_MAX 200
#define PRE_FLIGHT_MAX_OUTPUT 15

#define MAX_TILT 10

#define PIN_SERIAL_RX 2
#define PIN_SERIAL_TX 4

// Calibration values
// PID
#define KP 2
#define KI 5
#define KD 1

#define STABILITY_THRESHOLD 2
#define MMA7660_STABILITY_THRESHOLD 2
#define ITG3200_STABILITY_THRESHOLD 10
#define STABILIZATION_STEP 1

#define MMA7660_1G 40

#define DEBUG

// Flight modes
#define FLIGHT_MODE 0
#define AUTO_STABLE 0
#define AUTO_STABLE_INPUT_LIMIT 20
#define POINT_OK 0
#define POINT_HIGHER 1
#define POINT_LOWER 2

// }}}
// }}}
#endif
