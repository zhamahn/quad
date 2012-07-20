#include <Arduino.h>
#include "main.h"
#include "helpers.h"
#include "esc.h"
#include "itg3200.h"
#include "mma7660.h"
#include "ping.h"
#include <Wire.h>

ITG3200 Gyro;
MMA7660 Acc;
Ping Alt(PING_PIN);
ESC esc_x(ESC_X_PIN);
ESC esc_nx(ESC_NX_PIN);
ESC esc_y(ESC_Y_PIN);
ESC esc_ny(ESC_NY_PIN);
int i;
double output;
float angles[2];

int main(void)
{
  init();

#if defined(USBCON)
	USBDevice.attach();
#endif

	setup();
    
	for (;;) {
		loop();
		if (serialEventRun) serialEventRun();
	}
        
	return 0;
}

// {{{ Outputs
double avgOutput(void) {
  return (esc_x.output + esc_nx.output + esc_y.output + esc_ny.output) / 4;
}
void setOutputs(double output) {
  esc_x.set(output);
  esc_nx.set(output);
  esc_y.set(output);
  esc_ny.set(output);
}
void decreaseOutputs(int step = ESC_STEP) {
  esc_x.decrease(step);
  esc_nx.decrease(step);
  esc_y.decrease(step);
  esc_ny.decrease(step);
}
void increaseOutputs(int step = ESC_STEP) {
  esc_x.increase(step);
  esc_nx.increase(step);
  esc_y.increase(step);
  esc_ny.increase(step);
}
// }}}
// {{{ Mid-flight
char pointState(int point) {
  int angle = 0;
  switch (point) {
    case 0: angle = Acc.y; break;
    case 1: angle = Acc.x; break;
    case 2: angle = -Acc.y; break;
    case 3: angle = -Acc.x; break;
  }

  if ( angle < -STABILITY_THRESHOLD )
    return POINT_LOWER;
  else if ( angle > STABILITY_THRESHOLD )
    return POINT_HIGHER;
  else
    return POINT_OK;
}
bool altitudeIncreasing(void) { return (Acc.z > (MMA7660_1G + STABILITY_THRESHOLD)); }
bool altitudeDecreasing(void) { return (Acc.z < (MMA7660_1G - STABILITY_THRESHOLD)); }
bool XIsTilted(void) { return abs(Acc.x) > STABILITY_THRESHOLD; }
bool YIsTilted(void) { return abs(Acc.y) > STABILITY_THRESHOLD; }
bool isStable(void) { return (XIsTilted && YIsTilted); }
void stabilize(void) {
  Acc.read();
  Acc.print();
  while (!Acc.stable()) {
    if (Acc.y > STABILITY_THRESHOLD)      { esc_y.decrease(); esc_ny.increase(); }
    else if (Acc.y < STABILITY_THRESHOLD) { esc_y.increase(); esc_ny.decrease(); }

    if (Acc.x > STABILITY_THRESHOLD)      { esc_x.decrease(); esc_nx.increase(); }
    else if (Acc.x < STABILITY_THRESHOLD) { esc_x.increase(); esc_nx.decrease(); }

    Acc.read();
  }
}
// }}}
// {{{ Pre-flight
void setCorrections(void) {
  output = avgOutput();
  esc_x.correction  = esc_x.output  - output;
  esc_nx.correction = esc_nx.output - output;
  esc_y.correction  = esc_y.output  - output;
  esc_ny.correction = esc_ny.output - output;
}
void preFlightHalt(void) {
  debug("Something went wrong!, running preFlightHalt");

  bool motors_running = true;
  // Halt all motors
  while (motors_running) {
    motors_running = false;
    decreaseOutputs();
    output = avgOutput();
    if (output > OUTPUT_MIN)
      motors_running = true;
    delay(200);
  }

  // Enter infinite blocking loop
  while (true) {
    delay(1000);
  }
}

char preFlightHover(void) {
  debug("Entering preFlightHovering");

  bool do_loop = true;
  int counter = 0;
  char exit_code = 0;
  // Slowly increase all motor speeds until one side lifts up
  // then increase the motor speed on the side with lowest altitude
  while (counter < 4) {
    increaseOutputs(1);
    if (avgOutput() > PRE_FLIGHT_MAX_OUTPUT) {
      do_loop = false;
      exit_code++;
      break;
    }
    stabilize();
    //readAlt();
    //readAcc();
    /*if (distance > 10)*/
      /*do_loop = false;*/

    counter++;
    Serial.println("asd");
    delay(200);
  }

  return exit_code;
}

// Pre-flight sequence
void preFlight(void) {
  char return_codes = 0;
  debug("Running pre-flight setup");

  return_codes += preFlightHover();

  if (return_codes > 0)
    preFlightHalt();

  setCorrections();
}
// }}}
// {{{ Control functions
void computePIDs(void) {
  esc_x.pid->Compute();
  esc_nx.pid->Compute();
  esc_y.pid->Compute();
  esc_nx.pid->Compute();
}
void setOutputs(void) {
  esc_x.set();
  esc_nx.set();
  esc_y.set();
  esc_ny.set();
}
void stabilizeInput(void) {
  /*if (isStable()) {*/
    /*for (i=0; i<ESC_N; i++) {*/
      /*inputs[i] = setpoints[i];*/
    /*}*/
  /*} else {*/
    /*for (i=0; i<ESC_N; i++) {*/
      /*switch (pointState(i)) {*/
        /*case POINT_OK: inputs[i] = setpoints[i]; break;*/
        /*case POINT_HIGHER: inputs[i] = setpoints[i] - STABILITY_THRESHOLD; break;*/
        /*case POINT_LOWER: inputs[i] = setpoints[i] + STABILITY_THRESHOLD; break;*/
      /*}*/
    /*}*/
  /*}*/
}
void flightAutoStable(void) {
  stabilizeInput();
}
// }}}
// {{{ Input
void getInput(void) {
  switch (FLIGHT_MODE) {
    case AUTO_STABLE: flightAutoStable(); break;
  }
}
// }}}
// {{{ Main
void pingInterrupt(void) {
  Alt.measure();
}
void setup() {
  debug("Starting setup");
  Serial.begin(9600);
  Wire.begin();

  Acc.init();
  Gyro.init();

  attachInterrupt(PING_INT, pingInterrupt, FALLING);

  pinMode(ESC_X_PIN, OUTPUT);
  pinMode(ESC_NX_PIN, OUTPUT);
  pinMode(ESC_Y_PIN, OUTPUT);
  pinMode(ESC_NY_PIN, OUTPUT);

  // Set all motor speeds to ESC min
  setOutputs(OUTPUT_MIN);

  // Power on ESCs
  // batteryOn();
  // delay(ESC_STARTUP_DELAY);

  preFlight();

  debug("Entering main loop");
}

void loop() {
  int input;

  Acc.read();
  Acc.print();
  delay(100);

  getInput();
  //computePIDs();
  //setOutputs();

  if (Serial.available() > 0) {
    input = Serial.read();
    switch (input) {
      default: Serial.print("Input: "); Serial.println(input); break;
    }
  }

}
// }}}
