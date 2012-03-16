void Distance::init(int pin)
{
  _pin = pin;
}

int Distance::measure(void)
{
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(_pin, OUTPUT);
  digitalWrite(_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(_pin, LOW);
 
  // The same pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  return microsecondsToCentimeters(pulseIn(_pin, HIGH));
}

int Distance::microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}
