#ifndef ODOMETRY_H_
#define ODOMETRY_H_

#include "api.h"
#include "util.h"
#include "chassis_config.h"

const double ENCODER_TICKS = 360.0;
const double MIN_D_HEADING = 0.2;
const double PL_WHEEL_CIRC = 4.0*M_PI;
const double PD_WHEEL_CIRC = 4.0*M_PI;
const double PL_DIST = 10.44;
const double PD_DIST = 7.45;

void odometry();
void update_position();
void print_position();
void print_odometry_state();
double inertial_radians();
double pl_encoder_inches();
double pd_encoder_inches();
double get_robot_x();
double get_robot_y();
double get_robot_heading();

#endif // ODOMETRY_H_