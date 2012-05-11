// {{{ Includes
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
// }}}
// {{{ Defines
// Pin definitions
#define LED 13
#define PING_PIN 7

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

#define STABILITY_THRESHOLD 2

#define ULTRASONIC_MAX_RANGE 400

#define DEBUG

#define MMA7660addr   0x4c
#define MMA7660_X     0x00
#define MMA7660_Y     0x01
#define MMA7660_Z     0x02
#define MMA7660_TILT  0x03
#define MMA7660_SRST  0x04
#define MMA7660_SPCNT 0x05
#define MMA7660_INTSU 0x06
#define MMA7660_MODE  0x07
#define MMA7660_SR    0x08
#define MMA7660_PDET  0x09
#define MMA7660_PD    0x0A

#define ITG3200addr   0x68
#define ITG3200_WHO 	0x00
#define ITG3200_SMPL	0x15
#define ITG3200_DLPF	0x16
#define ITG3200_INT_C	0x17
#define ITG3200_INT_S	0x1A
#define ITG3200_TEMP_H	0x1B
#define ITG3200_TEMP_L	0x1C
#define ITG3200_GX_H	0x1D
#define ITG3200_GX_L	0x1E
#define ITG3200_GY_H	0x1F
#define ITG3200_GY_L	0x20
#define ITG3200_GZ_H	0x21
#define ITG3200_GZ_L	0x22
#define ITG3200_PWR_M	0x3E
// }}}
// {{{ Classes
struct ACC
{
  char x;
  char y;
  char z;
  char delta_x;
  char delta_y;
  char delta_z;
};

struct ROT
{
  int x;
  int y;
  int z;
};
// }}}
// {{{ Global variables
ACC Acc;
ROT Rot;
int Alt;
int i;
double speed;
// {{{ Per motor stuff
// Motor 0 (N)
double esc_0_input, esc_0_output, esc_0_setpoint;
Servo esc_0_servo;
PID esc_0_pid(&esc_0_input, &esc_0_output, &esc_0_setpoint, KP, KI, KD, AUTOMATIC);
int esc_0_correction;

// Motor 1 (E)
double esc_1_input, esc_1_output, esc_1_setpoint;
Servo esc_1_servo;
PID esc_1_pid(&esc_1_input, &esc_1_output, &esc_1_setpoint, KP, KI, KD, AUTOMATIC);
int esc_1_correction;

// Motor 2 (S)
double esc_2_input, esc_2_output, esc_2_setpoint;
Servo esc_2_servo;
PID esc_2_pid(&esc_2_input, &esc_2_output, &esc_2_setpoint, KP, KI, KD, AUTOMATIC);
int esc_2_correction;

// Motor 3 (W)
double esc_3_input, esc_3_output, esc_3_setpoint;
Servo esc_3_servo;
PID esc_3_pid(&esc_3_input, &esc_3_output, &esc_3_setpoint, KP, KI, KD, AUTOMATIC);
int esc_3_correction;

// }}}
// {{{ Arrays
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
double *inputs[MOTORS_N] =
{
  &esc_0_input,
  &esc_1_input,
  &esc_2_input,
  &esc_3_input
};
double *setpoints[MOTORS_N] =
{
  &esc_0_setpoint,
  &esc_1_setpoint,
  &esc_2_setpoint,
  &esc_3_setpoint
};

PID *pids[MOTORS_N] =
{
  &esc_0_pid,
  &esc_1_pid,
  &esc_2_pid,
  &esc_3_pid
};

int *corrections[MOTORS_N] =
{
  &esc_0_correction,
  &esc_1_correction,
  &esc_2_correction,
  &esc_3_correction
};
// }}}
// }}}
// {{{ Helpers
void debug(const char *msg)
{
  #ifdef DEBUG
    Serial.println(msg);
  #endif
}
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

void readReg(int dev, int reg, int count)
{
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(dev, count);
}
// }}}
// {{{ MMA7660
void accelInit(void)
{
  debug("Initializing accelerometer.");

  debug("--> Set to standby mode.");
  writeReg(MMA7660addr, MMA7660_MODE, 0x00);

  //debug("--> Set to generate automatic interrupt after every measurement");
  //writeReg(MMA7660addr, MMA7660_INTSU, 0x03);

  debug("--> Set sample rate.");
  writeReg(MMA7660addr, MMA7660_SR, 0x00);

  debug("--> Set pulse detection.");
  writeReg(MMA7660addr, MMA7660_PDET, 0x00);
  //writeReg(MMA7660addr, MMA7660_PDET, 0xE1);

  writeReg(MMA7660addr, MMA7660_PD, 0x04);
 
  debug("--> Set back to normal operation mode");
  writeReg(MMA7660addr, MMA7660_MODE, 0x01);
}

void readAcc(void)
{
  char val = 64;
  char data[3];
  readReg(MMA7660addr, MMA7660_X, 3);

  if (Wire.available())
  {
    for (i = 0; i < 3; i++)
    {
      while ( val > 63 ) // Values above 63 are invalid
        val = Wire.read();

      // transform the 7 bit signed number into an 8 bit signed number.
      data[i] = ((char)(val<<2));
      val = 64;
    }
  }

  Acc.delta_x = data[0] - Acc.x;
  Acc.delta_y = data[1] - Acc.y;
  Acc.delta_z = data[2] - Acc.z;
  Acc.x = data[0];
  Acc.y = data[1];
  Acc.z = data[2];
}
// }}}
// {{{ ITG3200
void gyroInit(void)
{
  debug("Initializing gyroscope.");

  debug("--> reset.");
  writeReg(ITG3200addr, ITG3200_PWR_M, 0x80);
 
  debug("--> set sample rate divider.");
  writeReg(ITG3200addr, ITG3200_SMPL, 0x00);
 
  debug("--> set measurement accuracy.");
  writeReg(ITG3200addr, ITG3200_DLPF, 0x18);
}

void readRot(void)
{
  readReg(ITG3200addr, ITG3200_GX_L, 6);

  if (Wire.available())
  {
    for (i = 0; i < 6; i++)
    {
      switch (i) {
        case 0: Rot.x  = Wire.read();    break;
        case 1: Rot.x |= Wire.read()<<8; break;
        case 2: Rot.y  = Wire.read();    break;
        case 3: Rot.y |= Wire.read()<<8; break;
        case 4: Rot.z  = Wire.read();    break;
        case 5: Rot.z |= Wire.read()<<8; break;
      }
    }
  }
}
// }}}
// {{{ IR Ping
void readAlt(void)
{
  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  Alt = pulseIn(PING_PIN, HIGH) / 29 / 2;
}
// }}}
// {{{ Sensors
void readSensors(void)
{
  //readAlt();
  //readRot();
  readAcc();
}

void printAcc(void)
{
  Serial.print("Acceleration: ");
  Serial.print("X: "); Serial.print(Acc.x, DEC);
  Serial.print(", DELTA_X: "); Serial.print(Acc.delta_x, DEC);
  Serial.print(", Y: "); Serial.print(Acc.y, DEC);
  Serial.print(", DELTA_Y: "); Serial.print(Acc.delta_y, DEC);
  Serial.print(", Z: "); Serial.print(Acc.z, DEC);
  Serial.print(", DELTA_z: "); Serial.print(Acc.delta_z, DEC);
  Serial.println("");
}

void printRot(void)
{
  Serial.print("Rotation: ");
  Serial.print("X: "); Serial.print(Rot.x, DEC);
  Serial.print(", Y: "); Serial.print(Rot.y, DEC);
  Serial.print(", Z: "); Serial.print(Rot.z, DEC);
  Serial.println("");
}

void printAlt(void)
{
  Serial.print("Altitude: "); Serial.print(Alt, DEC);
  Serial.println("");
}
// }}}
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
void stabilize(void)
{
  readSensors();
  Serial.println(Acc.y, DEC);
  while ((Acc.y < -STABILITY_THRESHOLD || Acc.y > STABILITY_THRESHOLD)
      && (Acc.z < -STABILITY_THRESHOLD || Acc.z > STABILITY_THRESHOLD)) {

    if (Acc.y > 0)
      increaseSpeed(0, true, ESC_SPEEDSTEP_PWM);
    else if (Acc.y < 0)
      increaseSpeed(2, true, ESC_SPEEDSTEP_PWM);
    if (Acc.z > 0)
      increaseSpeed(1, true, ESC_SPEEDSTEP_PWM);
    else if (Acc.z < 0)
      increaseSpeed(3, true, ESC_SPEEDSTEP_PWM);

    readSensors();
  }
}
// }}}
// {{{ Pre-flight
void preFlightHalt(void)
{
  debug("Something went wrong!, running preFlightHalt");

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
  debug("Entering preFlightHovering");

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
    stabilize();
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
// {{{ Main
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

  printAcc();
}
// }}}
