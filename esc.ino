void Esc::init(PID *_pid, Servo *_servo)
{
  debug("Initializing ESC");

  pid = _pid;
  servo = _servo;

  setpoint = 100;
  input = 0;
  output = 0;

  pid->SetMode(AUTOMATIC);
  setOutput(0);
}

void Esc::loopFunc(void)
{
  debug("Running ESC loopfunction");
  //pid->Compute();
  setOutput(input);
}

void Esc::setOutput(double value)
{
  if (value > 100)
    value = 100;
  else if (value < 0)
    value = 0;

  Serial.print("Setting output to: ");
  Serial.println(value, DEC);

  servo->writeMicroseconds(map(value, 0, 180, ESC_PWM_MIN, ESC_PWM_MAX));
  Serial.print("Current PWM: ");
  Serial.println(servo->readMicroseconds(), DEC);
}
