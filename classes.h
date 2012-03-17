struct Acceleration
{
  char x;
  char y;
  char z;
  char x_old;
  char y_old;
  char z_old;
};

struct Orientation
{
  char x;
  char y;
  char z;
  char tilt;
};

struct Rotation
{
  char x;
  char y;
  char z;
};

class Esc
{
  public:
    Esc(int, double, double, double, int);

    double input;
    double output;
    double setpoint;
    //void loopFunc(void);

    PID *pid;

  private:
    int pin;

    //void setOutput(void);
};

class Distance
{
  public:
    Distance(int);
    int measure(void);
    int cm;

  private:
    int pin;
    int microsecondsToCentimeters(long microseconds);
};
