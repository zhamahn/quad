#include <Wire.h>
 
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
 
struct Acceleration
{
 char x;
 char y;
 char z;
 char tilt;
};
 
void mma7660_init(void)
{
  // Set standby
  Wire.beginTransmission( MMA7660addr);
  Wire.write(byte(MMA7660_MODE));
  Wire.write(byte(0x00));
  Wire.endTransmission();

  Wire.beginTransmission( MMA7660addr);
  Wire.write(byte(MMA7660_MODE));   
  Wire.write(byte(0x07));
  Wire.endTransmission();

  Wire.beginTransmission( MMA7660addr);
  Wire.write(byte(MMA7660_SR));   
  Wire.write(byte(0x02));
  Wire.endTransmission();
 
  // Set active
  Wire.beginTransmission( MMA7660addr);
  Wire.write(byte(MMA7660_MODE));   
  Wire.write(byte(0x01));
  Wire.endTransmission();
 
}
 
void setup()
{
  Wire.begin();
  mma7660_init();
  Serial.begin(9600);
}
 
void Ecom()
{
  unsigned char val[4];
  int count = 0;
  val[0] = val[1] = val[2] = 64;
  Wire.requestFrom(0x4c, 4);    // request 3 bytes from slave device 0x4c
 
  while(Wire.available())  
  {
    if(count < 3)
    {
      while ( val[count] > 63 )  // reload the damn thing it is bad
      {
        val[count] = Wire.read();
      }
      count++;
    }
  }
 
  // transform the 7 bit signed number into an 8 bit signed number.
  Acceleration ret;
 
  ret.x = ((char)(val[0]<<2))/4;
  ret.y = ((char)(val[1]<<2))/4;
  ret.z = ((char)(val[2]<<2))/4;

  Serial.print("x = ");
  Serial.println(ret.x,DEC);   // print the reading

  Serial.print("y = ");
  Serial.println(ret.y,DEC);   // print the reading

  Serial.print("z = ");
  Serial.println(ret.z,DEC);   // print the reading
}

void updateAcc(void)
{
  Acceleration ret;
  unsigned char val[4] = { 64, 64, 64, 64 };
  char count = 0;

  // Set MMA7660addr to start reading from 0x00 (XOUT)
  Wire.beginTransmission(MMA7660addr);
  Wire.write(byte(MMA7660_X));
  Wire.endTransmission();

  // Request 3 bytes: X,Y,Z
  Wire.requestFrom(MMA7660addr, 3);

  while (Wire.available())
  {
    if (count < 4)
    {
      while ( val[count] > 63) // Values above 63 are incorrect
      {
        val[count] = Wire.read();
      }
      count++;
    }
  }
  
  ret.x = ((char)(val[0]<<2))/4;
  ret.y = ((char)(val[1]<<2))/4;
  ret.z = ((char)(val[2]<<2))/4;

  Serial.print(ret.x, DEC);
  Serial.print(", ");
  Serial.print(ret.y, DEC);
  Serial.print(", ");
  Serial.print(ret.z, DEC);
  Serial.print(", ");
  Serial.println(ret.tilt, BIN);

}

void loop()
{
  updateAcc();
  delay(50);
}
