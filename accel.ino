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

char * MMA7660ReadData(byte addr, int count)
{
  int i;
  char val[count+1];

  Wire.beginTransmission(MMA7660addr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, count);
  if (Wire.available())
  {
    for (i = 0; i < count; i++)
    {
      val[i] = ((char)(Wire.read()<<2))/4;
    }
  }
  return val;
}

void accelerationUpdate(struct Acceleration *acc)
{
  char *val;

  if (Interrupted)
  {
    Interrupted = false;

    val = MMA7660ReadData(MMA7660_X, 3);
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
  char *val;

  val = MMA7660ReadData(:MA7660_X, 4);
  ori->x = val[0];
  ori->y = val[1];
  ori->z = val[2];
  ori->tilt = val[3];
}
