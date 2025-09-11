#pragma once
#include <math.h>

// Wraps an angle to [0,360)
static inline float normalizeDeg(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

// Convert magnetic → true heading
extern float magneticDeclinationDeg;
static inline float magneticToTrue(float magDeg) {
  return normalizeDeg(magDeg + magneticDeclinationDeg);
}
