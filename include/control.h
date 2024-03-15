#ifndef CONTROL_H_
#define CONTROL_H_

/* Simple controller button/axis constants */
#define PROS_USE_SIMPLE_NAMES

#include "api.h"
#include "util.h"
#include "odometry.h"
#include "chassis_config.h"
#include "vision_config.h"

const double ROT_DEADZONE = 1.0;
const int MIN_CORRECTION_TICKS = 50;
const double CORRECTION_P = 60.0;
const double TAG_DIST = 5.0;

static pros::Controller controller(pros::E_CONTROLLER_MASTER);

void driver_control();
void holonomic_drive();
void handle_misc_input();
bool cancel_input();
void handle_tags();
void update_possession();
void tag();
void intake_control();
void flywheel_control();
void stop_drive();
void start_flywheel();
void stop_flywheel();
void intake();
void reverse_intake();
void stop_intake();
double get_power_scale(int num_powers, ...);
double distance_cm();

#endif // CONTROL_H_