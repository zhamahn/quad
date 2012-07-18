// {{{ Includes
#include <Wire.h>
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

#define OUTPUT_MIN 10
#define OUTPUT_MAX 200
#define ESC_SPEEDSTEP 3
#define PRE_FLIGHT_MAX_OUTPUT 15

// Calibration values
// PID
#define KP 2
#define KI 5
#define KD 1

#define STABILITY_THRESHOLD 2
#define STABILIZATION_STEP 1

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
double output;
volatile int alt;
volatile unsigned long ping_start = 0;
float angles[2];
// {{{ Per motor stuff
// Motor X
double esc_x_input, esc_x_output, esc_x_setpoint;
PID esc_x_pid(&esc_x_input, &esc_x_output, &esc_x_setpoint, KP, KI, KD, AUTOMATIC);
int esc_x_correction;

// Motor -X
double esc_nx_input, esc_nx_output, esc_nx_setpoint;
PID esc_nx_pid(&esc_nx_input, &esc_nx_output, &esc_nx_setpoint, KP, KI, KD, AUTOMATIC);
int esc_nx_correction;

// Motor Y
double esc_y_input, esc_y_output, esc_y_setpoint;
PID esc_y_pid(&esc_y_input, &esc_y_output, &esc_y_setpoint, KP, KI, KD, AUTOMATIC);
int esc_y_correction;

// Motor ny
double esc_ny_input, esc_ny_output, esc_ny_setpoint;
PID esc_ny_pid(&esc_ny_input, &esc_ny_output, &esc_ny_setpoint, KP, KI, KD, AUTOMATIC);
int esc_ny_correction;
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
// {{{ Output functions
double getOutput(int pin) {
  switch (pin) {
    case ESC_X_PIN: return esc_x_output; break;
    case ESC_NX_PIN: return esc_nx_output; break;
    case ESC_Y_PIN: return esc_y_output; break;
    case ESC_NY_PIN: return esc_ny_output; break;
  }
}
double avgOutput(void) {
  return (esc_x_output + esc_nx_output + esc_y_output + esc_ny_output) / 4;
}
double setOutput(int esc_pin, double output) {
  if (output < OUTPUT_MIN)
    output = OUTPUT_MIN;
  else if (output > OUTPUT_MAX)
    output = OUTPUT_MAX;

  analogWrite(esc_pin, output);
  return output;
}
void setOutput(double output) {
  setOutput(ESC_X_PIN,  output);
  setOutput(ESC_NX_PIN, output);
  setOutput(ESC_Y_PIN,  output);
  setOutput(ESC_NY_PIN, output);
}
double decreaseOutput(int pin, int step = ESC_SPEEDSTEP) {
  setOutput(pin, getOutput(pin) - step);
}
void decreaseOutput(int step = ESC_SPEEDSTEP) {
  setOutput(ESC_X_PIN,  esc_x_output  - step);
  setOutput(ESC_NX_PIN, esc_nx_output - step);
  setOutput(ESC_Y_PIN,  esc_y_output  - step);
  setOutput(ESC_NY_PIN, esc_ny_output - step);
}
double increaseOutput(int pin, int step = ESC_SPEEDSTEP) {
  setOutput(pin, getOutput(pin) - step);
}
void increaseOutput(int step = ESC_SPEEDSTEP) {
  setOutput(ESC_X_PIN,  esc_x_output  + step);
  setOutput(ESC_NX_PIN, esc_nx_output + step);
  setOutput(ESC_Y_PIN,  esc_y_output  + step);
  setOutput(ESC_NY_PIN, esc_ny_output + step);
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
      increaseOutput(ESC_NY_PIN, ESC_SPEEDSTEP);
    else if (Acc.y < 0)
      increaseOutput(ESC_Y_PIN, ESC_SPEEDSTEP);
    if (Acc.x > 0)
      increaseOutput(ESC_NX_PIN, ESC_SPEEDSTEP);
    else if (Acc.x < 0)
      increaseOutput(ESC_X_PIN, ESC_SPEEDSTEP);

    readAcc();
  }
}
// }}}
// {{{ Pre-flight
void setCorrections(void) {
  output = avgOutput();
  esc_x_correction  = esc_x_output  - output;
  esc_nx_correction = esc_nx_output - output;
  esc_y_correction  = esc_y_output  - output;
  esc_ny_correction = esc_ny_output - output;
}
void preFlightHalt(void) {
  debug("Something went wrong!, running preFlightHalt");

  bool motors_running = true;
  // Halt all motors
  while (motors_running) {
    motors_running = false;
    decreaseOutput();
    output = avgOutput();
    if (output > OUTPUT_MIN)
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
    increaseOutput(true, 1);
    if (avgOutput() > PRE_FLIGHT_MAX_OUTPUT) {
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
  setOutput(0);
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
  setOutput(ESC_X_PIN, esc_x_output);
  setOutput(ESC_NX_PIN, esc_x_output);
  setOutput(ESC_Y_PIN, esc_x_output);
  setOutput(ESC_NY_PIN, esc_x_output);
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

  pinMode(ESC_X_PIN, OUTPUT);
  pinMode(ESC_NX_PIN, OUTPUT);
  pinMode(ESC_Y_PIN, OUTPUT);
  pinMode(ESC_NY_PIN, OUTPUT);

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
