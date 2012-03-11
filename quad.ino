// Arduino libraries
#include <Wire.h>

// Pin definitions
#define LED 13
#define PING_PIN 7
#define ACC_INT_PIN 6

#define DEBUG

struct Acceleration
{
  char x;
  char y;
  char z;
  char x_old;
  char y_old;
  char z_old;
};

struct Orientation
{
  char x;
  char y;
  char z;
  char tilt;
};

struct Rotation
{
  char x;
  char y;
  char z;
};

void accelerationUpdate(struct Acceleration);
void accelInit(void);
void gyroInit(void);
void gyroUpdate(struct Rotation *);
Acceleration Acc;
Rotation Rot;
Orientation Ori;
volatile boolean Interrupted = false;

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

void setInterrupt(void)
{
  Interrupted = true;
}

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
  //debug("***************");
  accelerationUpdate(&Acc);
  //orientationUpdate(&Ori);
  Serial.print(Acc.x, DEC);
  Serial.print(", ");
  Serial.print(Acc.y, DEC);
  Serial.print(", ");
  Serial.println(Acc.z, DEC);

  orientationUpdate(&Ori);

  Serial.print(Ori.x, DEC);
  Serial.print(", ");
  Serial.print(Ori.y, DEC);
  Serial.print(", ");
  Serial.println(Ori.z, DEC);

  delay(50);
}
