// Arduino libraries
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>

// Other libraries
#include "headers.h"

// Pin definitions
#define LED 13
#define PING_PIN 7
#define ACC_INT_PIN 6

#define ESC_0_PIN 10
#define ESC_1_PIN 5
#define ESC_2_PIN 8
#define ESC_3_PIN 9

#define ESC_PWM_MIN 1180
#define ESC_PWM_MAX 1710

// PID calibration values
#define KP 2
#define KI 5
#define KD 1

#define DEBUG

Acceleration Acc;
Rotation Rot;
int distance;

// Motor 0 (N)
double esc_0_input, esc_0_output, esc_0_setpoint;
Servo esc_0_servo;
PID esc_0_pid(&esc_0_input, &esc_0_output, &esc_0_setpoint, KP, KI, KD, AUTOMATIC);

// Motor 1 (E)
double esc_1_input, esc_1_output, esc_1_setpoint;
Servo esc_1_servo;
PID esc_1_pid(&esc_1_input, &esc_1_output, &esc_1_setpoint, KP, KI, KD, AUTOMATIC);

// Motor 2 (S)
double esc_2_input, esc_2_output, esc_2_setpoint;
Servo esc_2_servo;
PID esc_2_pid(&esc_2_input, &esc_2_output, &esc_2_setpoint, KP, KI, KD, AUTOMATIC);

// Motor 3 (W)
double esc_3_input, esc_3_output, esc_3_setpoint;
Servo esc_3_servo;
PID esc_3_pid(&esc_3_input, &esc_3_output, &esc_3_setpoint, KP, KI, KD, AUTOMATIC);

volatile boolean Interrupted = false;

void setup()
{
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();
  accelInit();
  gyroInit();

  esc_0_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX)
  esc_1_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX)
  esc_2_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX)
  esc_3_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX)

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
