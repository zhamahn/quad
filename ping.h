#ifndef ping_h
#define ping_h
#define PING_MAX_RANGE 400
class Ping {
  public:
    volatile int distance;
    unsigned char pin;

    Ping(unsigned char);
    void start(void);
    void measure(void);
  
  private:
    volatile unsigned long started_at;
};
#endif
