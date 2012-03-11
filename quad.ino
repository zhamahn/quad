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

void accelInit(void);
void gyroInit(void);
void accelerationUpdate(struct Acceleration);
void rotationUpdate(struct Rotation *);
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

void serialPrintSensorValues(
  struct Acceleration *acc,
  struct Orientation *ori,
  struct Rotation *rot)
{
  //Serial.println("acc(x,y,z), ori(x,y,z), rot(x,y,z)");
  /*Serial.print(Acc.x, DEC);*/
  /*Serial.print(", ");*/
  /*Serial.print(Acc.y, DEC);*/
  /*Serial.print(", ");*/
  /*Serial.print(Acc.z, DEC);*/
  /*Serial.println("   |    ");*/

  /*Serial.print(Ori.x, DEC);*/
  /*Serial.print(", ");*/
  /*Serial.print(Ori.y, DEC);*/
  /*Serial.print(", ");*/
  /*Serial.print(Ori.z, DEC);*/
  /*Serial.print("   |    ");*/

  Serial.print(Rot.x, DEC);
  Serial.print(", ");
  Serial.print(Rot.y, DEC);
  Serial.print(", ");
  Serial.println(Rot.z, DEC);
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
  accelerationUpdate(&Acc);
  orientationUpdate(&Ori);
  rotationUpdate(&Rot);

  serialPrintSensorValues(&Acc, &Ori, &Rot);
  delay(100);
}
