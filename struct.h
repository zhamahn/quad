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
  void init(int);
  void setSpeed(int);

  int desired_throttle;
  int throttle;

  private:
  int pin;
};

class Distance
{
  public:
    void init(int);
    int measure(void);
    int cm;

  private:
    int pin;
    int microsecondsToCentimeters(long microseconds);
}
