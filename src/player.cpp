#include "player.h"

void follow_path() {
	double total_time = 1000.0;
	double cur_time = 0.0;
 	double update_ms = 10.0;

	while (cur_time <= total_time) { 
		double prev_time = cur_time;
		control_motors_path(cur_time);

		while (cur_time < prev_time + PATH_RATE) {
			cur_time += update_ms / 1000.0;
			update_position();
			pros::delay(update_ms);
		}
	}
	
	stop_drive();
}

void control_motors_path(double time_s) {
	Point pose_point = get_point(time_s + PATH_RATE);
	double pose_heading = get_heading(time_s + PATH_RATE);
	
	double dx = get_robot_x() - pose_point.x;
	double dy = pose_point.y - get_robot_y();
	double h = sqrt(dx*dx + dy*dy);
	
	double theta_i = -inertial_radians() + OFFSET;
	double d_theta = pose_heading - M_PI_2 + OFFSET - theta_i; 
	
	d_theta = -mod_angle(-d_theta);
	
	double thetaPath = atan2(dx, dy); 
    double redChord = -h*sin(thetaPath - (theta_i + d_theta/2)); 
    double greenChord = h*cos(thetaPath - (theta_i + d_theta/2)); 
    double leftArc = greenChord; 
    double rightArc = greenChord; 
    double topArc = redChord; 
    double bottomArc = redChord; 

    if (fabs(d_theta) > 0.002) {  // some funky magic number (straight line threshold)
        leftArc = (greenChord/(2*sin(d_theta/2)) - DRIVE_WHEEL_BASE/2)*d_theta; 
        rightArc = (greenChord/(2*sin(d_theta/2)) + DRIVE_WHEEL_BASE/2)*d_theta; 
        topArc = (redChord/(2*sin(d_theta/2)) - DRIVE_WHEEL_BASE/2)*d_theta; 
        bottomArc = (redChord/(2*sin(d_theta/2)) + DRIVE_WHEEL_BASE/2)*d_theta; 
    } 

	front_left_motor.move(-leftArc/PATH_RATE*PATH_P);
	back_right_motor.move(rightArc/PATH_RATE*PATH_P);
	front_right_motor.move(-topArc/PATH_RATE*PATH_P);
	back_left_motor.move(bottomArc/PATH_RATE*PATH_P);
}
