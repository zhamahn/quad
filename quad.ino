#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

#include "utils.h"
#include "esc.h"
#include "adxl345.h"
#include "ping.h"
#include "controller.h"
#include "control_center.h"
#include "hmc5883l.h"

// Pins
#define PING_PIN 7
#define PING_INT 0
#define PING_INT_PIN 2

#define ESC_X_PIN 6
#define ESC_NX_PIN 5
#define ESC_Y_PIN 10
#define ESC_NY_PIN 9

#define PIN_SERIAL_RX 4
#define PIN_SERIAL_TX 2

// Motor configuration
#define ESC_X_PITCH 0
#define ESC_NX_PITCH 0
#define ESC_Y_PITCH 1
#define ESC_NY_PITCH -1

#define ESC_X_ROLL 1
#define ESC_NX_ROLL -1
#define ESC_Y_ROLL 0
#define ESC_NY_ROLL 0

#define ESC_X_YAW 0.5
#define ESC_NX_YAW -0.5
#define ESC_Y_YAW 0.5
#define ESC_NY_YAW -0.5

#define ESC_X_ALTITUDE 1
#define ESC_NX_ALTITUDE 1
#define ESC_Y_ALTITUDE 1
#define ESC_NY_ALTITUDE 1

#define ESCS_COUNT 4

ITG3200 gyro;
ADXL345 acc;
HMC5883L mag;
Ping alt(PING_PIN);
ESC esc_x( ESC_X_PIN,  ESC_X_PITCH,  ESC_X_ROLL,  ESC_X_YAW,  ESC_X_ALTITUDE );
ESC esc_nx(ESC_NX_PIN, ESC_NX_PITCH, ESC_NX_ROLL, ESC_NX_YAW, ESC_NX_ALTITUDE);
ESC esc_y( ESC_Y_PIN,  ESC_Y_PITCH,  ESC_Y_ROLL,  ESC_Y_YAW,  ESC_Y_ALTITUDE );
ESC esc_ny(ESC_NY_PIN, ESC_NY_PITCH, ESC_NY_ROLL, ESC_NY_YAW, ESC_NY_ALTITUDE);
ESC *escs[ESCS_COUNT] = {&esc_x, &esc_nx, &esc_y, &esc_ny};
Controller controller;
ControlCenter control_center(escs, ESCS_COUNT);
SoftwareSerial mySerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
AHRS ahrs;

void pingInterrupt(void) {
  alt.measure();
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  Wire.begin();

  // Initialize classes
  acc.begin();
  gyro.begin();
  ahrs.begin();
  mag.begin();

  control_center.controller = &controller;
  control_center.ahrs       = &ahrs;
  control_center.gyro       = &gyro;
  control_center.acc        = &acc;
  control_center.mag        = &mag;

  ahrs.acc = &acc;
  ahrs.gyro = &gyro;
  ahrs.mag = &mag;

  controller.mySerial = &mySerial;

  attachInterrupt(PING_INT, pingInterrupt, FALLING);

  pinMode(ESC_X_PIN, OUTPUT);
  pinMode(ESC_NX_PIN, OUTPUT);
  pinMode(ESC_Y_PIN, OUTPUT);
  pinMode(ESC_NY_PIN, OUTPUT);
}

void loop() {
  controller.update();
  acc.update();
  gyro.update();
  mag.update();
  ahrs.update();
  control_center.update();
  //alt.start();
}
// }}}
