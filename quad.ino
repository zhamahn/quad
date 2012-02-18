// Arduino libraries
#include <Wire.h>

// Pin definitions
#define PING_PIN 7

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  Serial.println(ping());
}
