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

void gyroInit(void)
{
  debug("Initializing gyroscope.");

  debug("--> reset.");
  writeReg(ITG3200addr, ITG3200_PWR_M, 0x80);
 
  debug("--> set sample rate divider.");
  writeReg(ITG3200addr, ITG3200_SMPL, 0x00);
 
  debug("--> set measurement accuracy.");
  writeReg(ITG3200addr, ITG3200_DLPF, 0x18);
}

void ITG3200ReadData(char data[])
{
  int i;
  int size = 8;

  Wire.beginTransmission(ITG3200addr);
  Wire.write(ITG3200_GX_H);
  Wire.endTransmission();

  Wire.requestFrom(ITG3200addr, size);
  if (Wire.available())
  {
    for (i = 0; i < size; i++)
    {
      if (i%2 == 0)
        data[i/2] = Wire.read();
      else
        data[1/2] |= Wire.read()<<8;
    }
  }
}

void gyroRead(char * data, byte addr)
{
  Wire.beginTransmission(ITG3200addr);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(ITG3200addr,2);
  if (Wire.available()>0)
  {
    *data = Wire.read();
    *data |= Wire.read()<<8;
  }
}

void rotationUpdate(struct Rotation *rot)
{
  char data[4];
  ITG3200ReadData(data);

  /*gyro->x = gyroRead(ITG3200_GX_H);*/
  /*gyro->y = gyroRead(ITG3200_GY_H);*/
  /*gyro->z = gyroRead(ITG3200_GZ_H);*/
  rot->x = data[0];
  rot->y = data[1];
  rot->z = data[2];
}
