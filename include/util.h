#ifndef UTIL_H_
#define UTIL_H_

#include "math.h"

const int forward = 127;
const int reverse = -127;
const int stop = 0;
const double MAX_POWER = forward;
const double M_PI_180 = M_PI/180.0;

double mod_angle(double a);
int sign(int a);
int sign(double a);

#endif // UTIL_H_