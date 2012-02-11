#include <Servo.h> 
 
Servo esc;
int escSpeed = 0;
int speedStep = 5;
int escPin = 9;
int escMinPulseWidth = 1180;
int escMaxPulseWidth = 1710;
int serialInput;
int newSpeed;
 
void setup() 
{ 
  esc.attach(escPin, escMinPulseWidth, escMaxPulseWidth);
  Serial.begin(9600);
  setSpeed(&esc, escSpeed);
} 

void setSpeed(Servo *esc, int speed)
{
  int pulseWidth;

  if (speed < 0) { speed = 0; }
  if (speed > 100) { speed = 100; }

  Serial.print("Changing speed: ");
  Serial.print(escSpeed);

  pulseWidth = map(speed, 0, 100, escMinPulseWidth, escMaxPulseWidth);
  escSpeed = speed;
  esc->writeMicroseconds(pulseWidth);

  Serial.print(" -> ");
  Serial.println(escSpeed);
}
 
void loop() 
{ 
  int newSpeed;

  if (Serial.available() > 0) {
    serialInput = Serial.read();
    switch (serialInput) {
      case 49: newSpeed = 0; break; // 1
      case 50: newSpeed = 10; break; // 2
      case 51: newSpeed = 20; break; // 3
      case 52: newSpeed = 30; break; // 4
      case 53: newSpeed = 40; break; // 5
      case 54: newSpeed = 50; break; // 6
      case 55: newSpeed = 60; break; // 7
      case 56: newSpeed = 70; break; // 8
      case 57: newSpeed = 80; break; // 9
      case 48: newSpeed = 90; break; // 0
      case 100: newSpeed = escSpeed + speedStep; break;
      case 120: newSpeed = escSpeed - speedStep; break;
      default: Serial.print("Input: "); Serial.println(serialInput); break;
    }

    Serial.print("Current PWM width: ");
    Serial.println(esc.readMicroseconds());

    setSpeed(&esc, newSpeed);

    Serial.print("Current PWM width: ");
    Serial.println(esc.readMicroseconds());
  }
} 
