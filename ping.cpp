#include <Arduino.h>
#include "ping.h"

Ping::Ping(int _pin) {
  pin = _pin;
  started_at = 0;
}

void Ping::start(void) {
  if (started_at == 0) {
    noInterrupts(); // Disable interrupts, so interrupts wont trigger on the LOW-HIGH-LOW cycle

    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(5);
    digitalWrite(pin, LOW);

    // Wait the sensor to change pin to HIGH
    pinMode(pin, INPUT);
    while (digitalRead(pin) == LOW);

    interrupts(); // Re-enable interrupts
    started_at = micros();
  }
}

void Ping::measure(void) {
  if (started_at > 0) {
    distance = (micros() - started_at) / 29 / 2;
    started_at = 0;
  }
  if (distance > PING_MAX_RANGE)
    distance = PING_MAX_RANGE+1;
}

void Ping::print(void) {
  Serial.print("Altitude: ");
  Serial.print(distance, DEC);
  Serial.println("");
}

void Ping::interrupt(void) {
  measure();
}
