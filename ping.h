#ifndef ping_h
#define ping_h
#define PING_MAX_RANGE 400
class Ping {
  public:
  volatile int distance;
  int pin;

  Ping(int);
  void start(void);
  void measure(void);
  void print(void);
  
  private:
  volatile unsigned long started_at;
};
#endif
