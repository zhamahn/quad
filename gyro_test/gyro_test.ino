#include <Wire.h>
 
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

class GYRO
{
  public:
  char temp;
  char gx;
  char gy;
  char gz;
};

void ITG3200Init(void)
{
  Wire.beginTransmission(ITG3200addr);
  Wire.write(byte(0x3E));
  Wire.write(byte(0x80));  //write a reset to the device
  Wire.endTransmission(); //end transmission
 
  Wire.beginTransmission(ITG3200addr);
  Wire.write(byte(0x15));
  Wire.write(byte(0x00));   //sample rate divider
  Wire.endTransmission(); //end transmission
 
  Wire.beginTransmission(ITG3200addr);
  Wire.write(byte(0x16));
  Wire.write(byte(0x18)); // ±2000 degrees/s (default value)
  Wire.endTransmission(); //end transmission
 
//  Wire.beginTransmission(ITG3200addr);
//  Wire.write(byte(0x17));
//  Wire.write(byte(0x05));   // enable write raw values
//  Wire.endTransmission(); //end transmission
 
//  Wire.beginTransmission(ITG3200addr);
//  Wire.write(byte(0x3E));
//  Wire.write(byte(0x00));
//  Wire.endTransmission(); //end transmission
}

char ITG3200Readbyte(unsigned char address)
{
  char data;
  Wire.beginTransmission(ITG3200addr);
  Wire.write((address));
  Wire.endTransmission();
  Wire.requestFrom(ITG3200addr,1);
  if (Wire.available()>0)
  {
    data = Wire.read();
  }
  return data;
}
 
char ITG3200Read(unsigned char addressh,unsigned char addressl)
{
  char data;
 
  Wire.beginTransmission(ITG3200addr);
  Wire.write((addressh));
  Wire.endTransmission();
  Wire.requestFrom(ITG3200addr,1);
  if (Wire.available()>0)
  {
    data = Wire.read();
  }
  Wire.beginTransmission(ITG3200addr);
  Wire.write((addressl));
  Wire.endTransmission();
  if (Wire.available()>0)
  {
    data |= Wire.read()<<8;
  }
  return data;
}

GYRO Gyro;

void setup()
{
  Wire.begin();
  ITG3200Init();
  Serial.begin(9600);
}
 
void loop()
{
  Gyro.temp = ITG3200Read(ITG3200_TEMP_H, ITG3200_TEMP_L);
  Gyro.gx =   ITG3200Read(ITG3200_GX_H,   ITG3200_GX_L);
  Gyro.gy =   ITG3200Read(ITG3200_GY_H,   ITG3200_GY_L);
  Gyro.gx =   ITG3200Read(ITG3200_GZ_H,   ITG3200_GZ_L);

  int test;

  test = Gyro.gx + Gyro.gy;

  Serial.print(Gyro.gx, DEC);
  Serial.print(", ");
  Serial.print(Gyro.gy, DEC);
  Serial.print(" = ");
  Serial.println(test, DEC);
 
  //Serial.println(ITG3200Readbyte(ITG3200_WHO),HEX);    
  //Serial.println(ITG3200Readbyte(ITG3200_DLPF),BIN);  
  //Serial.println(ITG3200Readbyte(ITG3200_SMPL),BIN);  
 //
  //Serial.println(ITG3200Readbyte(ITG3200_PWR_M),BIN);    
 //
  //Serial.println("*************");
 
 
  delay(50);
}
