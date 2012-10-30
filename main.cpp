#include <Arduino.h>
#include <Wire/Wire.h>
#include <SoftwareSerial.h>
#include "main.h"
#include "helpers.h"
#include "esc.h"
#include "itg3200.h"
#include "mma7660.h"
#include "ping.h"
#include "controller.h"
#include "communication.h"
#include "escs.h"
#include "quad.h"

ITG3200 gyro;
MMA7660 acc;
Ping alt(PING_PIN);
ESC esc_x(ESC_X_PIN);
ESC esc_nx(ESC_NX_PIN);
ESC esc_y(ESC_Y_PIN);
ESC esc_ny(ESC_NY_PIN);
ESCs escs;
Controller controller;
Quad quad;
SoftwareSerial mySerial(PIN_SERIAL_RX, PIN_SERIAL_TX);

void pingInterrupt(void) {
  alt.measure();
}

int main(void)
{
  init();

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

void setup() {
  debug("Starting setup");
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

  Wire.begin();

  acc.init();
  gyro.init();

  attachInterrupt(PING_INT, pingInterrupt, FALLING);

  pinMode(ESC_X_PIN, OUTPUT);
  pinMode(ESC_NX_PIN, OUTPUT);
  pinMode(ESC_Y_PIN, OUTPUT);
  pinMode(ESC_NY_PIN, OUTPUT);

  // Set all motor speeds to ESC min
  escs.write(OUTPUT_MIN);

  debug("Entering main loop");
}

void loop() {
  unsigned char data[8];
  if (mySerial.available() > 0) {
    if (readFrame(&mySerial, data) == FRAME_COMPLETE)
      controller.updateFromDataArray(data);
  }

  acc.read();
  gyro.read();
  alt.start();

  quad.computePIDs();
  quad.setESCs();
}
// }}}
