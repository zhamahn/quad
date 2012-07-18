// {{{ Includes
#include <Wire.h>
#include <Servo.h>
#include <PID_v1.h>
// }}}
// {{{ Defines
// Pin definitions
#define LED 13
#define PING_PIN 7
#define PING_INT 0
#define PING_INT_PIN 2

#define ESC_X_PIN 10
#define ESC_NX_PIN 5
#define ESC_Y_PIN 8
#define ESC_NY_PIN 9

#define ESC_ALL -1
#define ESC_N 4

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
#define STABILIZATION_STEP 10

#define ULTRASONIC_MAX_RANGE 400
#define MMA7660_1G 40

#define DEBUG

// Flight modes
#define FLIGHT_MODE 0
#define AUTO_STABLE 0
#define AUTO_STABLE_INPUT_LIMIT 20
#define POINT_OK 0
#define POINT_HIGHER 1
#define POINT_LOWER 2

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
struct ACC {
  char x;
  char y;
  char z;
  char delta_x;
  char delta_y;
  char delta_z;
};

struct ROT {
  int x;
  int y;
  int z;
};
// }}}
// {{{ Global variables
ACC Acc;
ROT Rot;
int i;
double speed;
volatile int alt;
volatile unsigned long ping_start = 0;
// {{{ Per motor stuff
// Motor X
double esc_x_input, esc_x_output, esc_x_setpoint;
Servo esc_x;
PID esc_x_pid(&esc_x_input, &esc_x_output, &esc_x_setpoint, KP, KI, KD, AUTOMATIC);
int esc_x_correction;

// Motor -X
double esc_nx_input, esc_nx_output, esc_nx_setpoint;
Servo esc_nx;
PID esc_nx_pid(&esc_nx_input, &esc_nx_output, &esc_nx_setpoint, KP, KI, KD, AUTOMATIC);
int esc_nx_correction;

// Motor Y
double esc_y_input, esc_y_output, esc_y_setpoint;
Servo esc_y;
PID esc_y_pid(&esc_y_input, &esc_y_output, &esc_y_setpoint, KP, KI, KD, AUTOMATIC);
int esc_y_correction;

// Motor ny
double esc_ny_input, esc_ny_output, esc_ny_setpoint;
Servo esc_ny;
PID esc_ny_pid(&esc_ny_input, &esc_ny_output, &esc_ny_setpoint, KP, KI, KD, AUTOMATIC);
int esc_ny_correction;
// }}}
// {{{ Arrays
Servo *escs[ESC_N] = {
  &esc_x,
  &esc_nx,
  &esc_y,
  &esc_ny
};

double *outputs[ESC_N] = {
  &esc_x_output,
  &esc_nx_output,
  &esc_y_output,
  &esc_ny_output
};
double *inputs[ESC_N] = {
  &esc_x_input,
  &esc_nx_input,
  &esc_y_input,
  &esc_ny_input
};
double *setpoints[ESC_N] = {
  &esc_x_setpoint,
  &esc_nx_setpoint,
  &esc_y_setpoint,
  &esc_ny_setpoint
};

PID *pids[ESC_N] = {
  &esc_x_pid,
  &esc_nx_pid,
  &esc_y_pid,
  &esc_ny_pid
};

int *corrections[ESC_N] = {
  &esc_x_correction,
  &esc_nx_correction,
  &esc_y_correction,
  &esc_ny_correction
};
float angles[2];
// }}}
// }}}
// {{{ Helpers
void debug(const char *msg) {
  #ifdef DEBUG
    Serial.println(msg);
  #endif
}
void writeReg(byte dev, byte reg, byte val) {
  Wire.beginTransmission(dev);
  delay(100);
  Wire.write(reg);
  delay(10);
  Wire.write(val);
  delay(10);
  Wire.endTransmission();
}

void readReg(int dev, int reg, int count) {
  Wire.beginTransmission(dev);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(dev, count);
}
// }}}
// {{{ MMA7660
void accelInit(void) {
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

void readAcc(void) {
  char val = 64;
  char data[3];
  readReg(MMA7660addr, MMA7660_X, 3);

  if (Wire.available()) {
    for (i = 0; i < 3; i++) {
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
void gyroInit(void) {
  debug("Initializing gyroscope.");

  debug("--> reset.");
  writeReg(ITG3200addr, ITG3200_PWR_M, 0x80);
 
  debug("--> set sample rate divider.");
  writeReg(ITG3200addr, ITG3200_SMPL, 0x00);
 
  debug("--> set measurement accuracy.");
  writeReg(ITG3200addr, ITG3200_DLPF, 0x18);
}

void readRot(void) {
  readReg(ITG3200addr, ITG3200_GX_L, 6);

  if (Wire.available()) {
    for (i = 0; i < 6; i++) {
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
void startPing(void) {
  if (ping_start == 0) {
    noInterrupts(); // Disable interrupts, so interrupts wont trigger on the LOW-HIGH-LOW cycle

    pinMode(PING_PIN, OUTPUT);
    digitalWrite(PING_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(PING_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(PING_PIN, LOW);

    // Wait the sensor to change pin to HIGH
    pinMode(PING_PIN, INPUT);
    while (digitalRead(PING_PIN) == LOW);

    interrupts(); // Re-enable interrupts
    ping_start = micros();
  }
}
void measureAlt(void) {
  if (ping_start > 0) {
    alt = (micros() - ping_start) / 29 / 2;
    ping_start = 0;
  }
  if (alt > ULTRASONIC_MAX_RANGE)
    alt = ULTRASONIC_MAX_RANGE;
}
// }}}
// {{{ Sensors
void printAcc(void) {
  Serial.print("Acceleration: ");
  Serial.print("X: "); Serial.print(Acc.x, DEC);
  //Serial.print(", DELTA_X: "); Serial.print(Acc.delta_x, DEC);
  Serial.print(", Y: "); Serial.print(Acc.y, DEC);
  //Serial.print(", DELTA_Y: "); Serial.print(Acc.delta_y, DEC);
  Serial.print(", Z: "); Serial.print(Acc.z, DEC);
  //Serial.print(", DELTA_z: "); Serial.print(Acc.delta_z, DEC);
  Serial.println("");
}

void printRot(void) {
  Serial.print("Rotation: ");
  Serial.print("X: "); Serial.print(Rot.x, DEC);
  Serial.print(", Y: "); Serial.print(Rot.y, DEC);
  Serial.print(", Z: "); Serial.print(Rot.z, DEC);
  Serial.println("");
}

void printAlt(void) {
  Serial.print("Altitude: "); Serial.print(alt, DEC);
  Serial.println("");
}
// }}}
// {{{ Speed functions
double getSpeed(Servo * esc, bool pwm = false) {
  if (pwm)
    speed = esc->readMicroseconds();
  else
    speed = map(esc->readMicroseconds(), ESC_PWM_MIN, ESC_PWM_MAX, 0, 100);

  return speed;
}
int getAvgSpeed(bool pwm = false) {
  double speeds = 0;
  speeds += getSpeed(&esc_x, pwm);
  speeds += getSpeed(&esc_nx, pwm);
  speeds += getSpeed(&esc_y, pwm);
  speeds += getSpeed(&esc_ny, pwm);
  return speeds / ESC_N;
}
void _setSpeed(Servo * esc, double speed) {
  if (0 <= speed && speed <= 100)
    esc->writeMicroseconds(map(speed, 0, 100, ESC_PWM_MIN, ESC_PWM_MAX));
  else if ( ESC_PWM_MIN <= speed && speed <= ESC_PWM_MAX )
    esc->writeMicroseconds(speed);
}
double normalizeSpeed(double speed) {
  if (speed <= 0)
    speed = 0;
  else if (speed >= 100 && speed <= 200)
    speed = 100;
  else if (speed > 200 && speed < ESC_PWM_MIN )
    speed = ESC_PWM_MIN;
  else if (speed >= ESC_PWM_MAX)
    speed = ESC_PWM_MAX;

  return speed;
}
double setSpeed(Servo * esc, double new_speed) {
  new_speed = normalizeSpeed(new_speed);
  _setSpeed(esc, new_speed);
  return new_speed;
}
double setSpeed(double new_speed) {
  new_speed = normalizeSpeed(new_speed);
  _setSpeed(&esc_x, new_speed);
  _setSpeed(&esc_nx, new_speed);
  _setSpeed(&esc_y, new_speed);
  _setSpeed(&esc_nx, new_speed);
  return new_speed;
}
void decreaseSpeed(Servo * esc, bool pwm = false, int step = ESC_SPEEDSTEP_PCT) {
  setSpeed(esc, getSpeed(esc) - step);
}
void decreaseSpeed(bool pwm = false, int step = ESC_SPEEDSTEP_PCT) {
  setSpeed(&esc_x, getSpeed(&esc_x) - step);
  setSpeed(&esc_nx, getSpeed(&esc_nx) - step);
  setSpeed(&esc_y, getSpeed(&esc_y) - step);
  setSpeed(&esc_nx, getSpeed(&esc_ny) - step);
}
void increaseSpeed(Servo * esc, bool pwm = false, int step = ESC_SPEEDSTEP_PCT) {
  setSpeed(esc, getSpeed(esc) + step);
}
void increaseSpeed(bool pwm = false, int step = ESC_SPEEDSTEP_PCT) {
  setSpeed(&esc_x, getSpeed(&esc_x) + step);
  setSpeed(&esc_nx, getSpeed(&esc_nx) + step);
  setSpeed(&esc_y, getSpeed(&esc_y) + step);
  setSpeed(&esc_nx, getSpeed(&esc_ny) + step);
}
// }}}
// {{{ Mid-flight
char pointState(int point) {
  int angle = 0;
  switch (point) {
    case 0: angle = Acc.y; break;
    case 1: angle = Acc.x; break;
    case 2: angle = -Acc.y; break;
    case 3: angle = -Acc.x; break;
  }

  if ( angle < -STABILITY_THRESHOLD )
    return POINT_LOWER;
  else if ( angle > STABILITY_THRESHOLD )
    return POINT_HIGHER;
  else
    return POINT_OK;
}
bool altitudeIncreasing(void) { return (Acc.z > (MMA7660_1G + STABILITY_THRESHOLD)); }
bool altitudeDecreasing(void) { return (Acc.z < (MMA7660_1G - STABILITY_THRESHOLD)); }
bool NSIsTilted(void) { return (Acc.y < -STABILITY_THRESHOLD || Acc.y > STABILITY_THRESHOLD); }
bool WEIsTilted(void) { return (Acc.z < -STABILITY_THRESHOLD || Acc.z > STABILITY_THRESHOLD); }
bool isStable(void) { return (NSIsTilted && WEIsTilted); }
void stabilize(void) {
  readAcc();
  Serial.println(Acc.y, DEC);
  while (NSIsTilted || WEIsTilted) {
    printAcc();
    if (Acc.y > 0)
      increaseSpeed(&esc_ny, true, ESC_SPEEDSTEP_PWM);
    else if (Acc.y < 0)
      increaseSpeed(&esc_y, true, ESC_SPEEDSTEP_PWM);
    if (Acc.x > 0)
      increaseSpeed(&esc_nx, true, ESC_SPEEDSTEP_PWM);
    else if (Acc.x < 0)
      increaseSpeed(&esc_x, true, ESC_SPEEDSTEP_PWM);

    readAcc();
  }
}
// }}}
// {{{ Pre-flight
void setCorrections(void) {
  int avg;

  avg = getAvgSpeed(true);
  for (i=0; i<ESC_N; i++)
    *corrections[i] = (escs[i]->readMicroseconds() - avg);
}
void preFlightHalt(void) {
  debug("Something went wrong!, running preFlightHalt");

  bool motors_running = true;
  // Halt all motors
  while (motors_running) {
    motors_running = false;
    decreaseSpeed();
    speed = getAvgSpeed();
    if (speed > 10)
      motors_running = true;
    delay(200);
  }

  // Enter infinite blocking loop
  while (true) {
    delay(1000);
  }
}

char preFlightHover(void) {
  debug("Entering preFlightHovering");

  bool do_loop = true;
  int counter = 0;
  char exit_code = 0;
  // Slowly increase all motor speeds until one side lifts up
  // then increase the motor speed on the side with lowest altitude
  while (counter < 4) {
    increaseSpeed(true, 1);
    if (getAvgSpeed() > PRE_FLIGHT_MAX_OUTPUT) {
      do_loop = false;
      exit_code++;
      break;
    }
    stabilize();
    //readAlt();
    //readAcc();
    /*if (distance > 10)*/
      /*do_loop = false;*/

    counter++;
    Serial.println("asd");
    delay(200);
  }

  return exit_code;
}

// Pre-flight sequence
void preFlight(void) {
  char return_codes = 0;
  debug("Running pre-flight setup");

  // Set all motor speeds to 0
  setSpeed(0);
  return_codes += preFlightHover();

  if (return_codes > 0)
    preFlightHalt();

  setCorrections();
}
// }}}
// {{{ Control functions
void computePIDs(void) {
  esc_x_pid.Compute();
  esc_nx_pid.Compute();
  esc_y_pid.Compute();
  esc_nx_pid.Compute();
}
void setOutputs(void) {
  setSpeed(&esc_x, esc_x_output);
  setSpeed(&esc_nx, esc_nx_output);
  setSpeed(&esc_y, esc_y_output);
  setSpeed(&esc_ny, esc_ny_output);
}
void stabilizeInput(void) {
  /*if (isStable()) {*/
    /*for (i=0; i<ESC_N; i++) {*/
      /*inputs[i] = setpoints[i];*/
    /*}*/
  /*} else {*/
    /*for (i=0; i<ESC_N; i++) {*/
      /*switch (pointState(i)) {*/
        /*case POINT_OK: inputs[i] = setpoints[i]; break;*/
        /*case POINT_HIGHER: inputs[i] = setpoints[i] - STABILITY_THRESHOLD; break;*/
        /*case POINT_LOWER: inputs[i] = setpoints[i] + STABILITY_THRESHOLD; break;*/
      /*}*/
    /*}*/
  /*}*/
}
void flightAutoStable(void) {
  stabilizeInput();
}
// }}}
// {{{ Input
void getInput(void) {
  switch (FLIGHT_MODE) {
    case AUTO_STABLE: flightAutoStable(); break;
  }
}
// }}}
// {{{ Main
void setup() {
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();
  accelInit();
  gyroInit();

  attachInterrupt(PING_INT, measureAlt, FALLING);

  esc_x.attach(ESC_X_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_nx.attach(ESC_NX_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_y.attach(ESC_Y_PIN, ESC_PWM_MIN, ESC_PWM_MAX);
  esc_ny.attach(ESC_NY_PIN, ESC_PWM_MIN, ESC_PWM_MAX);

  //preFlight();

  debug("Entering main loop");
}

void loop() {
  int input;

  //preFlight();
  readAcc();
  printAcc();
  delay(100);

  getInput();
  //computePIDs();
  //setOutputs();

  if (Serial.available() > 0) {
    input = Serial.read();
    switch (input) {
      default: Serial.print("Input: "); Serial.println(input); break;
    }
  }

}
// }}}
