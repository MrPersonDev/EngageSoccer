#ifndef PLAYER_H_
#define PLAYER_H_

#include "math.h"
#include "control.h"
#include "odometry.h"
#include "path.h"

const double PATH_LINEAR_P = 10.0;
const double PATH_ANGULAR_P = 60.0;
const double LOOK_AHEAD_DIST = 12.0;

void follow_path();
void control_motors_path();

#endif // PLAYER_H_