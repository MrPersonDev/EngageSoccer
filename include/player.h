#ifndef PLAYER_H_
#define PLAYER_H_

#include "math.h"
#include "control.h"
#include "odometry.h"
#include "path.h"

const double DRIVE_WHEEL_BASE = 17.0;
const double PATH_P = 1.0;
const double PATH_RATE = 0.5;

void follow_path();
void control_motors_path(double time_s);

#endif // PLAYER_H_