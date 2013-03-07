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
  Wire.begin();
  quad.begin();
  acc.begin();
  gyro.begin();
  dcm.begin();

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
  unsigned long loopStart = micros();
  unsigned char data[8];
  if (mySerial.available() > 0) {
    if (readFrame(&mySerial, data) == FRAME_COMPLETE)
      controller.updateFromDataArray(data);
  }

  /*char input;*/
  /*bool isKi = false;*/
  /*bool isKp = false;*/
  /*bool valRead = false;*/
  /*char val_string[5];*/
  /*int i;*/
  /*for (i=0; i<10; i++) { val_string[i] = '0'; }*/
  /*int val_index = 0;*/
  /*while (Serial.available() > 0) {*/
    /*input = Serial.read();*/
    /*if (isKi && valRead) {*/
      /*dcm.Ki = atof(val_string);*/
    /*} else if (isKp && valRead) {*/
      /*dcm.Kp = atof(val_string);*/
    /*} else if (isKp || isKi) {*/
      /*if (input == '\n') {*/
        /*valRead = true;*/
      /*} else {*/
        /*val_string[val_index] = input;*/
        /*val_index += 1;*/
      /*}*/
    /*} else {*/
      /*switch (input) {*/
      /*case 'p': isKp = true; break;*/
      /*case 'i': isKi = true; break;*/
      /*}*/
    /*}*/
  /*}*/

  acc.update();
  gyro.update();
  dcm.update();
  //alt.start();

  //quad.computePIDs();
  /*quad.setESCs();*/
  /*acc.printForGraph(); Serial.print('\t');*/
  /*gyro.printForGraph(); Serial.print('\t');*/
  /*dcm.printForGraph();*/
  /*Serial.print(dcm.q0, DEC); Serial.print(",");*/
  /*Serial.print(dcm.q1, DEC); Serial.print(",");*/
  /*Serial.print(dcm.q2, DEC); Serial.print(",");*/
  /*Serial.print(dcm.q3, DEC);*/
  Serial.print(dcm.pitch, DEC); Serial.print('\t');
  Serial.print(dcm.roll, DEC); Serial.print('\t');
  Serial.print(dcm.yaw, DEC); Serial.print('\t');
  unsigned long loopEnd = micros();
  Serial.print(" Loop execution time: ");
  Serial.print(loopEnd - loopStart, DEC);
  Serial.println("");
  delay(50);
}
// }}}
