#include "goalie.h"

int ball_side = 0;

void track_ball() {
	double forward_rand = rand() / (double)RAND_MAX;
	const double forward_pw = 50.0;
	const double forward_lb = 0.005;
	const double forward_ms = 500.0;
	if (forward_rand <= forward_lb && ball_side == 0) {
		front_left_motor.move(-forward_pw);
		front_right_motor.move(forward_pw);
		back_left_motor.move(-forward_pw);
		back_right_motor.move(forward_pw);
		intake();
		pros::delay(forward_ms);
		front_left_motor.move(stop);
		front_right_motor.move(stop);
		back_left_motor.move(stop);
		back_right_motor.move(stop);
		stop_intake();
	}

	pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
	double mid_x = rtn.x_middle_coord;
	const double cam_mid_x = 130.0;

	if (fabs(mid_x-cam_mid_x) < 30.0) {
		mid_x = cam_mid_x;
		ball_side = 0;
	} else if (rtn.signature == VISION_NO_SIG) {
		if (ball_side == -1) {
			mid_x = 0;
		} else if (ball_side == 1) {
			mid_x = 200;
		} else if (ball_side == 0) {
			mid_x = cam_mid_x;
		}
	} else {
		if (mid_x < 40) {
			ball_side = -1;
		} else if (mid_x > 200) {
			ball_side = 1;
		}
	}
	
	const double TRACK_P = 0.9;
	const double TRACK_ANGULAR_P = 90.0;
	
	double linear_dif = cam_mid_x - mid_x;
	double linear_pw = linear_dif * TRACK_P;
	
	double rot_dif = mod_angle(inertial_radians());
	double rot_pw = rot_dif * TRACK_ANGULAR_P;
	
	double fl_power = linear_pw + rot_pw;
	double fr_power = linear_pw + rot_pw;
	double bl_power = -linear_pw + rot_pw;
	double br_power = -linear_pw + rot_pw;
	
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
