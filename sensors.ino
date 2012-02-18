// Gyroscope address & registrys
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

// Acceleration sensor address & registrys
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

struct GYRO
{
  char temp;
  char x;
  char y;
  char z;
};

struct ACC
{
 char tilt;
 char x;
 char y;
 char z;
};

// Functions for acceleration sensor
void MMA7660_init(void);
void updateAcc(ACC *acc);

// Functions for gyroscope
void ITG3200_init(void);
void updateGyro(GYRO *gyro);

// Functions for distance sensor
long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

long ping(void)
{
  long duration;

  pinMode(PING_PIN, OUTPUT);
  digitalWrite(PING_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(PING_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(PING_PIN, LOW);

  pinMode(PING_PIN, INPUT);
  duration = pulseIn(PING_PIN, HIGH);

  return microsecondsToCentimeters(duration);
}
