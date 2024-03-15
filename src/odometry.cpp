#include "odometry.h"

double robot_x = 0.0, robot_y = 0.0;
double robot_heading = 0.0;
double prev_x = 0.0, prev_y = 0.0;
double prev_heading = 0.0;

void odometry() {
	while (true) {
		update_position();
		pros::delay(10.0);
	}
}

void update_position() {
    double cur_x = pd_encoder_inches();
    double cur_y = pl_encoder_inches();

    double d_ex = (cur_x - prev_x);
    double d_ey = (cur_y - prev_y);
	
    double heading = -inertial_radians();

    double d_heading = heading - prev_heading;
    prev_heading = heading;

    double dxInertial = d_ex;
    double dyInertial = d_ey;
    if (fabs(d_heading) > MIN_D_HEADING) {
        double hx = 2*(d_ex/d_heading - PL_DIST)*sin(d_heading/2);
		double hy = 2*(d_ey/d_heading - PD_DIST)*sin(d_heading/2);

        dxInertial = -hy*sin(d_heading/2) + hx*cos(d_heading/2);
        dyInertial = hy*cos(d_heading/2) + hx*sin(d_heading/2);
    }

    robot_x += -dyInertial*sin(heading) + dxInertial*cos(heading);
    robot_y += dyInertial*cos(heading) + dxInertial*sin(heading);
	robot_heading = heading;

	prev_x = cur_x;
    prev_y = cur_y;
}

void print_position() {
	char x_str[20], y_str[20], h_str[30];
	sprintf(x_str, "X: %.2f", robot_x);
	sprintf(y_str, "Y: %.2f", robot_y);
	sprintf(h_str, "H: %.2f deg", robot_heading/M_PI_180);
	pros::lcd::set_text(1, x_str);
	pros::lcd::set_text(2, y_str);
	pros::lcd::set_text(3, h_str);
}

void print_odometry_state() {
	char pl_str[20], pd_str[20];
	sprintf(pd_str, "PL: %.2f", pl_encoder.get_value() / ENCODER_TICKS);
	sprintf(pl_str, "PD: %.2f", pd_encoder.get_value() / ENCODER_TICKS);
	pros::lcd::set_text(1, pl_str);
	pros::lcd::set_text(1, pl_str);
	pros::lcd::set_text(2, pd_str);
}

double inertial_radians() {
	return inertial_sensor.get_heading() * M_PI_180;
}

double pl_encoder_inches() {
	return (pl_encoder.get_value() / ENCODER_TICKS) * PL_WHEEL_CIRC;
}

double pd_encoder_inches() {
	return (pd_encoder.get_value() / ENCODER_TICKS) * PD_WHEEL_CIRC;
}

double get_robot_x() {
	return robot_x;
}

double get_robot_y() {
	return robot_y;
}

double get_robot_heading() {
	return robot_heading;
}
