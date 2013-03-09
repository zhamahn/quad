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

ITG3200 gyro;
ADXL345 acc;
HMC5883L mag;
Ping alt(PING_PIN);
ESC esc_x(ESC_X_PIN);
ESC esc_nx(ESC_NX_PIN);
ESC esc_y(ESC_Y_PIN);
ESC esc_ny(ESC_NY_PIN);
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
  Wire.begin();

  // Initialize classes
  quad.begin();
  acc.begin();
  gyro.begin();
  dcm.begin();

  quad.controller = &controller;
  quad.dcm        = &dcm;

  dcm.acc = &acc;
  dcm.gyro = &gyro;
  dcm.mag = &mag;

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
  dcm.update();

  quad.updatePIDs();
  quad.setOutputs();
  //alt.start();
}
// }}}
