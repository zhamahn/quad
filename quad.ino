// Arduino libraries
#include <Wire.h>
#include <PID_v1.h>
#include <Servo.h>

// Other libraries
#include "classes.h"

// Pin definitions
#define LED 13
#define PING_PIN 7
#define ACC_INT_PIN 6

#define ESC_N 6
#define ESC_E 7
#define ESC_S 8
#define ESC_W 9

#define DEBUG

Acceleration Acc;
Rotation Rot;
Orientation Ori;
Distance Dis(PING_PIN);
volatile boolean Interrupted = false;

void setup()
{
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();
  accelInit();
  //gyroInit();

  pinMode(ACC_INT_PIN, INPUT);

  attachInterrupt(0, setInterrupt, CHANGE);
  debug("Entering main loop");
}

void loop()
{
  accelerationUpdate(&Acc);
  orientationUpdate(&Ori);
  rotationUpdate(&Rot);

  serialPrintSensorValues(&Acc, &Ori, &Rot);
  delay(100);
}
