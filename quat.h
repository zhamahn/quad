#ifndef quat_h
#define quat_h
class Quat {
  public:
    float w, i, j, k;

    Quat(void);
    Quat(Quat*);
    
    float pitch(void);
    float roll(void);
    float yaw(void);

    float rotX(void);
    float rotY(void);
    float rotZ(void);

    void normalize(void);
    void invert(void);
};

void q_product(Quat*, Quat*, Quat*);
#endif
