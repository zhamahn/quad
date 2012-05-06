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

void MMA7660ReadData(char data[], int count, byte addr)
{
  int i;

  Wire.beginTransmission(MMA7660addr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, count);
  if (Wire.available())
  {
    for (i = 0; i < count; i++)
    {
      data[i] = ((char)(Wire.read()<<2))/4;
    }
  }
}

void readAccel(struct Acceleration *acc)
{
  char data[3];

  MMA7660ReadData(data, 3, MMA7660_X);
  acc->x = data[0] - acc->x_old;
  acc->y = data[1] - acc->y_old;
  acc->z = data[2] - acc->z_old;
  acc->x_old = data[0];
  acc->y_old = data[1];
  acc->z_old = data[2];
}
