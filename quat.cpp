#include <Arduino.h>
#include "quat.h"

Quat::Quat(void) {
  w = 1.0;
  i = 0.0;
  j = 0.0;
  k = 0.0;
}

Quat::Quat(Quat *orig) {
  w = orig->w;
  i = orig->i;
  j = orig->j;
  k = orig->k;
}

void Quat::invert(void) {
  i *= -1;
  j *= -1;
  k *= -1;
}

void Quat::normalize(void) {
  float norm;
  norm = sqrt(w*w + i*i + j*j + k*k);
  w /= norm;
  i /= norm;
  j /= norm;
  k /= norm;
}

float Quat::pitch(void) {
  return asin(2*(w*j - i*k));
}

float Quat::roll(void) {
  return rotX();
}

float Quat::yaw(void) {
  return rotZ();
}

float Quat::rotY(void) {
  return atan2(2*(w*j + i*k), 1 - 2*(i*i + j*j));
}

float Quat::rotX(void) {
  return atan2(2*(w*i + j*k), 1 - 2*(i*i + j*j));
}

float Quat::rotZ(void) {
  return atan2(2*(w*k + i*j), 1 - 2*(j*j + k*k));
}

void q_product(Quat *product, Quat *quat0, Quat *quat1) {
  product->w = quat0->w*quat1->w - quat0->i*quat1->i - quat0->j*quat1->j - quat0->k*quat1->k;
  product->i = quat0->w*quat1->i + quat0->i*quat1->w + quat0->j*quat1->k - quat0->k*quat1->j;
  product->j = quat0->w*quat1->j - quat0->i*quat1->k + quat0->j*quat1->w + quat0->k*quat1->i;
  product->k = quat0->w*quat1->k + quat0->i*quat1->j - quat0->j*quat1->i + quat0->k*quat1->w;
}
