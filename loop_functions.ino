void readSensors(void)
{
  //distance = ping(PING_PIN);
  readAccel();
  #ifdef DEBUG
    Serial.print("ACC x: ");
    Serial.print(acceleration.x, DEC);
    Serial.print(", ACC y: ");
    Serial.print(acceleration.y, DEC);
    Serial.print(", ACC z: ");
    Serial.print(acceleration.z, DEC);
    Serial.print(", ACC delta_x: ");
    Serial.print(acceleration.delta_x, DEC);
    Serial.print(", ACC delta_y: ");
    Serial.print(acceleration.delta_y, DEC);
    Serial.print(", ACC delta_z: ");
    Serial.println(acceleration.delta_z, DEC);
  #endif
}
