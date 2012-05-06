// Function prototypes
int ping(int);

void readSensors(void);
void readInput(void);
void computePID(void);
void setOutputs(void);

// Class definitions
struct Acceleration
{
  char x;
  char y;
  char z;
  char x_old;
  char y_old;
  char z_old;
  char delta_x;
  char delta_y;
  char delta_z;
};

struct Rotation
{
  int x;
  int y;
  int z;
};