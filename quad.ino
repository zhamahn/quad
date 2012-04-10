// Arduino libraries
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Other libraries
#include "classes.h"

// Pin definitions
#define LED 13
#define PING_PIN 7
#define ACC_INT_PIN 6

#define ESC_N_PIN 10
#define ESC_E_PIN 5
#define ESC_S_PIN 8
#define ESC_W_PIN 9

#define ESC_N 0
#define ESC_E 1
#define ESC_S 2
#define ESC_W 3

#define ESC_PWM_MIN 1180
#define ESC_PWM_MAX 1710

// PID calibration values
#define KP 2
#define KI 5
#define KD 1

#define DEBUG

Acceleration Acc;
Rotation Rot;
Orientation Ori;
Distance Dis(PING_PIN);

// ESC N
Esc EscN;
Servo EscNServo;
PID EscNPID(&EscN.input, &EscN.output, &EscN.setpoint, KP, KI, KD, AUTOMATIC);

// ESC E
Esc EscE;
Servo EscEServo;
PID EscEPID(&EscE.input, &EscE.output, &EscE.setpoint, KP, KI, KD, AUTOMATIC);

// ESC S
Esc EscS;
Servo EscSServo;
PID EscSPID(&EscS.input, &EscS.output, &EscS.setpoint, KP, KI, KD, AUTOMATIC);

// ESC W
Esc EscW;
Servo EscWServo;
PID EscWPID(&EscW.input, &EscW.output, &EscW.setpoint, KP, KI, KD, AUTOMATIC);

Esc *Escs[4];

volatile boolean Interrupted = false;
double pulseWidth = ESC_PWM_MIN;
int CurrentEsc;

void setup()
{
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();
  //accelInit();
  //gyroInit();
  EscNServo.attach(ESC_N_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  EscN.init(&EscNPID, &EscNServo);

  EscEServo.attach(ESC_E_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  EscE.init(&EscEPID, &EscEServo);

  EscSServo.attach(ESC_S_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  EscS.init(&EscSPID, &EscSServo);

  EscWServo.attach(ESC_W_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  EscW.init(&EscWPID, &EscWServo);

  Escs[ESC_N] = &EscN;
  Escs[ESC_E] = &EscE;
  Escs[ESC_S] = &EscS;
  Escs[ESC_W] = &EscW;

  CurrentEsc = ESC_N;

  pinMode(ACC_INT_PIN, INPUT);
  attachInterrupt(0, setInterrupt, CHANGE);
  debug("Entering main loop");
}

void loop()
{
  int input;

  if (Serial.available() > 0) {
    input = Serial.read();
    switch (input) {
      case 49: CurrentEsc = ESC_N; break;
      case 50: CurrentEsc = ESC_E; break;
      case 51: CurrentEsc = ESC_S; break;
      case 52: CurrentEsc = ESC_W; break;
      //case 108: Escs[CurrentEsc]->input += 10; break;
      //case 115: Escs[CurrentEsc]->input -= 10; break;
      case 108: pulseWidth += 10; break;
      case 115: pulseWidth -= 10; break;
      default: Serial.print("Input: "); Serial.println(input); break;
    }

    //Serial.println(Escs[CurrentEsc]->input);

    Serial.println(CurrentEsc, DEC);
    //Escs[CurrentEsc]->loopFunc();
    Escs[CurrentEsc]->servo->writeMicroseconds(pulseWidth);
    Serial.println(Escs[CurrentEsc]->servo->readMicroseconds(), DEC);
    //Serial.println(EscNServo.readMicroseconds());
  }

  //Serial.println(input, DEC);

  //Serial.print(EscN.input, DEC);
  //Serial.print(", ");
  //EscN.loopFunc();
  //Serial.println(EscN.setpoint, DEC);

  //accelerationUpdate(&Acc);
  //orientationUpdate(&Ori);
  //rotationUpdate(&Rot);

  //serialPrintSensorValues(&Acc, &Ori, &Rot);
  delay(100);
}
