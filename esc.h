#ifndef esc_h
#define esc_h

#define OUTPUT_MIN 10
#define OUTPUT_MAX 255

class ESC {
  public:
    unsigned char pin;
    int gain;
    int output;

    ESC(unsigned char);

    int write(void);
    int set(int);
    int change(int);
};

#endif
