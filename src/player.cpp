#include "player.h"

void follow_path() {
    set_points_curvature();
    const double update_ms = 10.0;
    while (true) {
        control_motors_path();
        update_position();
        pros::delay(update_ms);
    }
}

void control_motors_path() {
    double cur_x = get_robot_x();
    double cur_y = get_robot_y();
    double heading = get_robot_heading();

    Point target_point = get_point(cur_x, cur_y, LOOK_AHEAD_DIST);
    double linear_response = (1.0 - target_point.curvature) * PATH_LINEAR_P;

    double target_heading = 0;

    double d_x = target_point.x - cur_x;
    double d_y = target_point.y - cur_y;
    double d_heading = mod_angle(target_heading - heading);

    double rot_response = d_heading * PATH_ANGULAR_P;

	double magnitude = sqrt(d_x*d_x + d_y*d_y) * linear_response;
	double angle = atan2(d_x, d_y);
	double new_x = magnitude * sin(angle + heading);
	double new_y = magnitude * cos(angle + heading);

	double fr_power = new_x * cos(A1) + new_y * sin(A1) + rot_response; 
	double fl_power = new_x * cos(A2) + new_y * sin(A2) + rot_response; 
	double bl_power = new_x * cos(A3) + new_y * sin(A3) + rot_response; 
	double br_power = new_x * cos(A4) + new_y * sin(A4) + rot_response; 

	double power_scale = get_power_scale(4, fr_power, fl_power, bl_power, br_power);
	fr_power *= power_scale;
	fl_power *= power_scale;
	bl_power *= power_scale;
	br_power *= power_scale;

	front_left_motor.move(fl_power);
	front_right_motor.move(fr_power);
	back_left_motor.move(bl_power);
	back_right_motor.move(br_power);
}
