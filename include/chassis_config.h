#ifndef CHASSIS_CONFIG_H_
#define CHASSIS_CONFIG_H_

#include "api.h"
#include "vision_config.h"

const int FRONT_LEFT_MOTOR_PORT = 20;
const int FRONT_RIGHT_MOTOR_PORT = 6;
const int BACK_LEFT_MOTOR_PORT = 7;
const int BACK_RIGHT_MOTOR_PORT = 8;
const int FLYWHEEL_MOTOR_PORT_1 = 1;
const int FLYWHEEL_MOTOR_PORT_2 = 5;
const int INTAKE_MOTOR_PORT_1 = 10;
const int INERTIAL_PORT = 19;
const int VISION_PORT = 2;

static pros::Motor front_left_motor(FRONT_LEFT_MOTOR_PORT, true);
static pros::Motor front_right_motor(FRONT_RIGHT_MOTOR_PORT, true);
static pros::Motor back_left_motor(BACK_LEFT_MOTOR_PORT, true);
static pros::Motor back_right_motor(BACK_RIGHT_MOTOR_PORT, true);
static pros::Motor intake_motor_1(INTAKE_MOTOR_PORT_1, true);
static pros::Motor flywheel_motor_1(FLYWHEEL_MOTOR_PORT_1, false);
static pros::Motor flywheel_motor_2(FLYWHEEL_MOTOR_PORT_2, true);

static pros::IMU inertial_sensor(INERTIAL_PORT);
static pros::ADIEncoder pl_encoder('A', 'B', false);
static pros::ADIEncoder pd_encoder('C', 'D', false);
static pros::ADIUltrasonic dist_sensor('G', 'H');

static pros::Vision vision_sensor(VISION_PORT);

/* Wheel angles */
const double A1 = 135 * M_PI / 180; 
const double A2 = 225 * M_PI / 180; 
const double A3 = 315 * M_PI / 180; 
const double A4 = 45  * M_PI / 180; 

/* Encoder wheel offset from drive wheels */
const double OFFSET = -M_PI/4.0;

void initialize_robot();

#endif // CHASSIS_CONFIG_H_
