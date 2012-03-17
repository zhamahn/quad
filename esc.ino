#define ESC_PWM_MIN 1180
#define ESC_PWM_MAX 1710

Esc::Esc(int _pin, double Kp, double Ki, double Kd, int mode)
{
  pin = _pin;

  pid = new PID(&input, &output, &setpoint, Kp, Ki, Kd, mode);

  setpoint = 100;
  input = 0;
  output = 0;
  pid->SetMode(AUTOMATIC);
}
