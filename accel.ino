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

  debug("--> Set to generate automatic interrupt after every measurement");
  writeReg(MMA7660addr, MMA7660_INTSU, 0x03);

  debug("--> Set sample rate.");
  writeReg(MMA7660addr, MMA7660_SR, 0x00);

  debug("--> Set pulse detection.");
  writeReg(MMA7660addr, MMA7660_PDET, 0x00);
  //writeReg(MMA7660addr, MMA7660_PDET, 0xE1);

  writeReg(MMA7660addr, MMA7660_PD, 0x04);
 
  debug("--> Set back to normal operation mode");
  writeReg(MMA7660addr, MMA7660_MODE, 0x01);
}

void MMA7660ReadData(char * val)
{
  int i;
  int size = 4;

  Wire.beginTransmission(MMA7660addr);
  Wire.write(byte(MMA7660_X));
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, size);
  if (Wire.available())
  {
    for (i = 0; i < size; i++)
    {
      val[i] = ((char)(Wire.read()<<2))/4;
    }
  }
}

void accelerationUpdate(struct Acceleration *acc)
{
  char val[4] = { 64, 64, 64 };

  if (Interrupted)
  {
    Interrupted = false;

    MMA7660ReadData(val);
    acc->x = val[0] - acc->x_old;
    acc->y = val[1] - acc->y_old;
    acc->z = val[2] - acc->z_old;
    acc->x_old = val[0];
    acc->y_old = val[1];
    acc->z_old = val[2];
  }
}

void orientationUpdate(struct Orientation *ori)
{
  char val[4] = { 64, 64, 64, 64 };

  MMA7660ReadData(val);
  ori->x = val[0];
  ori->y = val[1];
  ori->z = val[2];
  ori->tilt = val[3];
}
