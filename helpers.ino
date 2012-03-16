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
