#include "control.h"

bool in_possession = false;
double target_heading = 0.0;
int ticks_without_rot = 0;

void driver_control() {
	while (true) {
		update_possession();
		handle_tags();
		holonomic_drive();
		handle_misc_input();
		intake_control();
		flywheel_control();
		
		pros::delay(10.0);
	}
}

void holonomic_drive() {
	int x = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
	int y = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	double rot = 0;
	rot = -controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

	if (fabs(rot) < ROT_DEADZONE) {
		ticks_without_rot++;
		if (ticks_without_rot > MIN_CORRECTION_TICKS) {
			rot = mod_angle(target_heading - get_robot_heading()) * CORRECTION_P;
		} else {
			target_heading = get_robot_heading();
		}
	} else {
		ticks_without_rot = 0;
	}

	double magnitude = sqrt(x*x + y*y);
	double angle = atan2(x, y);
	double inertial_rotation = inertial_sensor.get_rotation() * M_PI / 180.0;
	double new_x = magnitude * sin(angle - inertial_rotation);
	double new_y = magnitude * cos(angle - inertial_rotation);

	double fr_power = new_x * cos(A1) + new_y * sin(A1) + rot; 
	double fl_power = new_x * cos(A2) + new_y * sin(A2) + rot; 
	double bl_power = new_x * cos(A3) + new_y * sin(A3) + rot; 
	double br_power = new_x * cos(A4) + new_y * sin(A4) + rot; 

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

void handle_misc_input() {
	if (controller.get_digital_new_press(DIGITAL_LEFT)) {
		inertial_sensor.set_heading(0);
	}
}

bool cancel_input() {
	return controller.get_digital(DIGITAL_X);
}

void handle_tags() {
	if (distance_cm() <= TAG_DIST && in_possession) {
		tag();
	}
}

void update_possession() {
	pros::vision_object_s_t rtn = vision_sensor.get_by_size(0);
	in_possession = rtn.signature != VISION_NO_SIG;
}

void tag() {
	controller.rumble("...");
	reverse_intake();
	stop_flywheel();
	stop_drive();
	pros::delay(5000.0);
}

void intake_control() {
	if (controller.get_digital(DIGITAL_L2)) {
		intake();
	} else if (controller.get_digital(DIGITAL_L1)) {
		reverse_intake();
	} else {
		stop_intake();
	}
}

void flywheel_control() {
	if (controller.get_digital_new_press(DIGITAL_R2)) {
		start_flywheel();
	} else if (controller.get_digital_new_press(DIGITAL_R1)) {
		stop_flywheel();
	}
}

void stop_drive() {
	front_left_motor.move(stop);
	front_right_motor.move(stop);
	back_left_motor.move(stop);
	back_right_motor.move(stop);
}

void start_flywheel() {
	flywheel_motor_1.move(forward);
	flywheel_motor_2.move(forward);
}

void stop_flywheel() {
	flywheel_motor_1.move(stop);
	flywheel_motor_2.move(stop);
}

void intake() {
	intake_motor_1.move(forward);
}

void reverse_intake() {
	intake_motor_1.move(reverse);
}

void stop_intake() {
	intake_motor_1.move(stop);
}

double get_power_scale(int num_powers, ...) {
    va_list args;
    va_start(args, num_powers);

    double max_element = 0.0;
    for (int i = 0; i < num_powers; ++i) {
        double p = va_arg(args, double);
        max_element = std::max(max_element, std::abs(p));
    }
    va_end(args);

    if (max_element > MAX_POWER) return MAX_POWER / max_element;
    else return 1.0;
}

double distance_cm() {
	return dist_sensor.get_value() / 10.0;
}
