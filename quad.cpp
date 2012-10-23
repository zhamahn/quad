#include <Arduino.h>
#include "quad.h"
#include "main.h"

void Quad::stabilize(void) {
  acc->read();
  while (!acc->stable()) {
    if (acc->y > STABILITY_THRESHOLD)      { escs->y->decrease(); escs->ny->increase(); }
    else if (acc->y < STABILITY_THRESHOLD) { escs->y->increase(); escs->ny->decrease(); }

    if (acc->x > STABILITY_THRESHOLD)      { escs->x->decrease(); escs->nx->increase(); }
    else if (acc->x < STABILITY_THRESHOLD) { escs->x->increase(); escs->nx->decrease(); }

    acc->read();
  }
}

void Quad::landNow(void) {
  stabilize();

  // Halt all motors
  while (! escs->allStopped()) {
    escs->decreaseOutputs();
    delay(100);
  }
}

void Quad::setAltitude(int altitude) {
  // craft is below desired altitude
  if (alt->distance < altitude) {
    while (alt->distance < altitude)
      escs->increaseOutputs();
  } else {
  // craft is above desired altitude
    while (alt->distance > altitude)
      escs->decreaseOutputs();
  }
}

void Quad::preFlight(void) {
  debug("Running pre-flight setup");
  setAltitude(20);
  escs->setCorrections();
}
// }}}
