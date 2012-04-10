#include <Servo.h> 
 
Servo esc;
int escSpeed = 0;
int speedStep = 5;
int escPin = 9;
int escMinPulseWidth = 1180;
int escMaxPulseWidth = 1710;
int serialInput;
int newSpeed;
int pulseWidth;
 
void setup() 
{ 
  esc.attach(escPin, escMinPulseWidth, escMaxPulseWidth);
  Serial.begin(9600);
  pulseWidth = escMinPulseWidth;
  setSpeed(&esc, escSpeed);
} 

void setSpeed(Servo *esc, int speed)
{
  esc->writeMicroseconds(speed);

  Serial.print(" -> ");
  Serial.println(escSpeed);
}
 
void loop() 
{ 
  int newSpeed;

  if (Serial.available() > 0) {
    serialInput = Serial.read();
    switch (serialInput) {
      case 49: pulseWidth = pulseWidth + speedStep; break;
      case 50: pulseWidth = pulseWidth - speedStep; break;
      default: Serial.print("Input: "); Serial.println(serialInput); break;
    }

    Serial.print("Current PWM width: ");
    Serial.println(esc.readMicroseconds());

    setSpeed(&esc, pulseWidth);

    Serial.print("Current PWM width: ");
    Serial.println(esc.readMicroseconds());
  }
} 
