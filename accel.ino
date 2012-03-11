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

  //writeReg(MMA7660addr, MMA7660_PD, 0x01);
 
  debug("--> Set back to normal operation mode");
  writeReg(MMA7660addr, MMA7660_MODE, 0x01);

  /*pinMode(ACC_INT_PIN, OUTPUT);*/
  /*digitalWrite(ACC_INT_PIN, HIGH);*/
  /*pinMode(ACC_INT_PIN, INPUT);*/
}

void MMA7660ReadRegistry(char * vals)
{
  int bailout = 3;
  int count = 4;
  int i;

  Wire.beginTransmission(MMA7660addr);
  Wire.write(byte(MMA7660_X));
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, count);
  if (Wire.available())
  {
    /*if (count < 3)*/
    /*{*/
      /*while ( val[count] > 63) // Values above 63 are incorrect*/
      /*{*/
        /*val[count] = Wire.read();*/
        /*if (bailout < 0)*/
        /*{*/
          /*debug("BAILOUT");*/
          /*break;*/
        /*} else {*/
          /*bailout--;*/
        /*}*/
      /*}*/
      /*bailout = 3;*/
      /*count++;*/
    /*}*/
    for (i = 0; i < count; i++)
    {
      vals[count] = ((char)(Wire.read()<<2))/4;
    }
  }
}

void accelerationUpdate(struct Acceleration *acc)
{
  unsigned char val[4] = { 64, 64, 64, 64 };
  char count = 0;
  int bailout = 3;

  if (Interrupted)
  {
    Interrupted = false;

    Wire.beginTransmission(MMA7660addr);
    Wire.write(byte(MMA7660_X));
    Wire.endTransmission();

    // Request 3 bytes: X,Y,Z
    Wire.requestFrom(MMA7660addr, 4);
    if (Wire.available())
    {
      /*if (count < 3)*/
      /*{*/
        /*while ( val[count] > 63) // Values above 63 are incorrect*/
        /*{*/
          /*val[count] = Wire.read();*/
          /*if (bailout < 0)*/
          /*{*/
            /*debug("BAILOUT");*/
            /*break;*/
          /*} else {*/
            /*bailout--;*/
          /*}*/
        /*}*/
        /*bailout = 3;*/
        /*count++;*/
      /*}*/
      for (count = 0; count < 3; count++)
      {
        val[count] = Wire.read();
      }
    }
    acc->x = ((char)(val[0]<<2))/4 - acc->x_old;
    acc->y = ((char)(val[1]<<2))/4 - acc->y_old;
    acc->z = ((char)(val[2]<<2))/4 - acc->z_old;
    acc->x_old = ((char)(val[0]<<2))/4;
    acc->y_old = ((char)(val[1]<<2))/4;
    acc->z_old = ((char)(val[2]<<2))/4;
  }
}

void orientationUpdate(struct Orientation *ori)
{
  unsigned char val[4] = { 64, 64, 64, 64 };
  char count = 0;
  int bailout = 3;
  boolean interrupt = false;

  Wire.beginTransmission(MMA7660addr);
  Wire.write(byte(MMA7660_X));
  Wire.endTransmission();

  Wire.requestFrom(MMA7660addr, 4);
  if (Wire.available())
  {
    /*if (count < 3)*/
    /*{*/
      /*while ( val[count] > 63) // Values above 63 are incorrect*/
      /*{*/
        /*val[count] = Wire.read();*/
        /*if (bailout < 0)*/
        /*{*/
          /*debug("BAILOUT");*/
          /*break;*/
        /*} else {*/
          /*bailout--;*/
        /*}*/
      /*}*/
      /*bailout = 3;*/
      /*count++;*/
    /*}*/
    for (count = 0; count < 4; count++)
    {
      val[count] = Wire.read();
    }
  }
  ori->x = ((char)(val[0]<<2))/4;
  ori->y = ((char)(val[1]<<2))/4;
  ori->z = ((char)(val[2]<<2))/4;
  ori->tilt = ((char)(val[3]<<2))/4;
}
