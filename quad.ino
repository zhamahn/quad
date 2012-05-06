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

#define MOTORS_N 4
#define MOTORS_ALL -1

#define ESC_PWM_MIN 1180
#define ESC_PWM_MAX 1710
#define ESC_SPEEDSTEP_PCT 1
#define ESC_SPEEDSTEP_PWM 10
#define PRE_FLIGHT_MAX_OUTPUT 15

// Calibration values
// PID
#define KP 2
#define KI 5
#define KD 1

#define BALANCE_THRESHOLD 2

#define DEBUG

// {{{ Global variables
Acceleration acceleration;
Rotation rotation;
int distance;
int i;
double speed;

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

Servo *servos[MOTORS_N] =
{
  &esc_0_servo,
  &esc_1_servo,
  &esc_2_servo,
  &esc_3_servo
};

double *outputs[MOTORS_N] =
{
  &esc_0_output,
  &esc_1_output,
  &esc_2_output,
  &esc_3_output
};

// }}}
// {{{ Funtions
void writeReg(byte dev, byte reg, byte val)
{
  Wire.beginTransmission(dev);
  delay(100);
  Wire.write(reg);
  delay(10);
  Wire.write(val);
  delay(10);
  Wire.endTransmission();
}

void debug(const char *msg)
{
  #ifdef DEBUG
    Serial.println(msg);
  #endif
}
// {{{ Speed functions
double getSpeed(int esc, bool pwm = false)
{
  if (pwm)
    speed = servos[esc]->readMicroseconds();
  else
    speed = map(servos[esc]->readMicroseconds(), ESC_PWM_MIN, ESC_PWM_MAX, 0, 100);

  return speed;
}
double getAvgSpeed(bool pwm = false)
{
  double speeds = 0;
  for (i=0; i<MOTORS_N; i++)
    speeds += getSpeed(i, pwm);
  return speeds / MOTORS_N;
}
double setSpeed(int esc, double new_speed)
{
  // Normalize speed for incorrect input
  if (speed <= 0)
    new_speed = 0;
  else if (new_speed >= 100 && new_speed <= 200)
    new_speed = 100;
  else if (new_speed > 200 && new_speed < ESC_PWM_MIN )
    new_speed = ESC_PWM_MIN;
  else if (new_speed >= ESC_PWM_MAX)
    new_speed = ESC_PWM_MAX;
    
  // Write new_speed to esc
  if (0 <= new_speed && new_speed <= 100)
    servos[esc]->writeMicroseconds(map(new_speed, 0, 100, ESC_PWM_MIN, ESC_PWM_MAX));
  else if ( ESC_PWM_MIN <= new_speed && new_speed <= ESC_PWM_MAX )
    servos[esc]->writeMicroseconds(new_speed);

  *outputs[esc] = new_speed;
  #ifdef DEBUG
    Serial.print("Setting ");
    Serial.print(esc, DEC);
    Serial.print(" speed to: ");
    Serial.println(new_speed, DEC);
  #endif
  return new_speed;
}
double decreaseSpeed(int esc, bool pwm = false, int amount = ESC_SPEEDSTEP_PCT)
{
  if (esc == MOTORS_ALL)
    for(i=0; i<MOTORS_N; i++)
      speed = setSpeed(i, getSpeed(i) - amount);
  else
    speed = setSpeed(esc, getSpeed(esc) - amount);

  return speed;
}
double increaseSpeed(int esc, bool pwm = false, int amount = ESC_SPEEDSTEP_PCT)
{
  if (esc == MOTORS_ALL)
    for(i=0; i<MOTORS_N; i++)
      speed = setSpeed(i, getSpeed(i) + amount);
  else
    speed = setSpeed(esc, getSpeed(esc) + amount);

  return speed;
}
// }}}
// {{{ Mid-flight
void balance(void)
{
  readSensors();
  Serial.println(acceleration.y, DEC);
  while (acceleration.y < -BALANCE_THRESHOLD || acceleration.y > BALANCE_THRESHOLD) {
    if (acceleration.y > 0)
      increaseSpeed(0, true, ESC_SPEEDSTEP_PWM);
    else if (acceleration.y < 0)
      increaseSpeed(2, true, ESC_SPEEDSTEP_PWM);

    readSensors();
  }
  Serial.println(acceleration.z, DEC);
  while (acceleration.z < -BALANCE_THRESHOLD || acceleration.z > BALANCE_THRESHOLD) {
    if (acceleration.z > 0)
      increaseSpeed(1, true, ESC_SPEEDSTEP_PWM);
    else if (acceleration.z < 0)
      increaseSpeed(3, true, ESC_SPEEDSTEP_PWM);

    readSensors();
  }
}
// }}}
// {{{ Pre-flight

void preFlightHalt(void)
{
  debug("Something went wrong!, running pre_flight_halt");

  bool motors_running = true;
  // Halt all motors
  while (motors_running) {
    motors_running = false;
    for (i=0; i<MOTORS_N; i++) {
      speed = decreaseSpeed(i);
      if (speed > 0)
        motors_running = true;
    }
    delay(200);
  }

  // Enter infinite blocking loop
  while (true) {
    delay(1000);
  }
}

char preFlightHover(void)
{
  debug("Entering pre flight hovering");

  bool do_loop = true;
  int counter = 0;
  char exit_code = 0;
  // Slowly increase all motor speeds until one side lifts up
  // then increase the motor speed on the side with lowest altitude
  while (counter < 4) {
    for (i=0; i<MOTORS_N; i++) {
      increaseSpeed(i, true, 1);
    }
    if (getAvgSpeed() > PRE_FLIGHT_MAX_OUTPUT) {
      do_loop = false;
      exit_code++;
      break;
    }
    balance();
    readSensors();
    /*if (distance > 10)*/
      /*do_loop = false;*/

    counter++;
    Serial.println("asd");
    delay(200);
  }

  return exit_code;
}

// Pre-flight sequence
void preFlight(void)
{
  char return_codes = 0;
  debug("Running pre-flight setup");

  // Set all motor speeds to 0
  for (i=0; i<MOTORS_N; i++) {
    setSpeed(i, 0);
  }
  return_codes += preFlightHover();

  if (return_codes > 0)
    preFlightHalt();
}
// }}}
// }}}

void setup()
{
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();
  accelInit();
  gyroInit();

  esc_0_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_1_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_2_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_3_servo.attach(ESC_0_PIN, ESC_PWM_MIN, ESC_PWM_MAX);

  //preFlight();

  debug("Entering main loop");
}

void loop()
{
  int input;

  readSensors();
  //readInput();
  //computePID();
  //setOutputs();

  if (Serial.available() > 0) {
    input = Serial.read();
    switch (input) {
      default: Serial.print("Input: "); Serial.println(input); break;
    }
  }

  /*Serial.println(acceleration.x, DEC);*/
  /*Serial.println(acceleration.y, DEC);*/
  /*Serial.println(acceleration.z, DEC);*/
  delay(100);
}
