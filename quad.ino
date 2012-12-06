#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <PID_v1.h>

#include "quad.h"
#include "utils.h"
#include "esc.h"
#include "adxl345.h"
#include "ping.h"
#include "controller.h"
#include "communication.h"
#include "escs.h"
#include "control_center.h"

ITG3200 gyro;
ADXL345 acc;
Ping alt(PING_PIN);
ESC esc_x(ESC_X_PIN);
ESC esc_nx(ESC_NX_PIN);
ESC esc_y(ESC_Y_PIN);
ESC esc_ny(ESC_NY_PIN);
ESCs escs;
Controller controller;
ControlCenter quad;
SoftwareSerial mySerial(PIN_SERIAL_RX, PIN_SERIAL_TX);
DCM dcm;

void pingInterrupt(void) {
  alt.measure();
}

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);

  // Initialize classes
  escs.x  = &esc_x;
  escs.nx = &esc_nx;
  escs.y  = &esc_y;
  escs.ny = &esc_ny;

  quad.escs       = &escs;
  quad.controller = &controller;
  quad.gyro       = &gyro;
  quad.acc        = &acc;
  quad.alt        = &alt;
  quad.dcm        = &dcm;

  Wire.begin();

  acc.begin();
  gyro.begin();

  dcm.acc = &acc;
  dcm.gyro = &gyro;

  attachInterrupt(PING_INT, pingInterrupt, FALLING);

  pinMode(ESC_X_PIN, OUTPUT);
  pinMode(ESC_NX_PIN, OUTPUT);
  pinMode(ESC_Y_PIN, OUTPUT);
  pinMode(ESC_NY_PIN, OUTPUT);

  // Set all motor speeds to ESC min
  escs.set(OUTPUT_MIN);
}

void loop() {
  unsigned char data[8];
  if (mySerial.available() > 0) {
    if (readFrame(&mySerial, data) == FRAME_COMPLETE)
      controller.updateFromDataArray(data);
  }

  acc.update();
  gyro.update();
  alt.start();

  quad.computePIDs();
  quad.setESCs();
}
// }}}
