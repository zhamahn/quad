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

void MMA7660ReadData(char val[], char count, byte addr)
{
  Wire.beginTransmission(MMA7660addr);
  Wire.write(addr);
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, count);
  if (Wire.available())
  {
    for (i = 0; i < count; i++)
    {
      val[i] = 64;
      while ( val[i] > 63 ) // Values above 63 are invalid
      {
        val[i] = Wire.read();
      }
    }
  }
}

void readAccel(void)
{
  char val[3];

  MMA7660ReadData(val, 3, MMA7660_X);
  // transform the 7 bit signed number into an 8 bit signed number.
  for (i=0; i < 3; i++)
    //val[i] = ((char)(val[i]<<2))/4;
    val[i] = ((char)(val[i]<<2));

  acceleration.x = val[0];
  acceleration.y = val[1];
  acceleration.z = val[2];
  acceleration.delta_x = val[0] - acceleration.x_old;
  acceleration.delta_y = val[1] - acceleration.y_old;
  acceleration.delta_z = val[2] - acceleration.z_old;
  acceleration.x_old = val[0];
  acceleration.y_old = val[1];
  acceleration.z_old = val[2];
}
