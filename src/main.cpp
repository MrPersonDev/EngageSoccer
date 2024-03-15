#include "main.h"

#include "chassis_config.h"
#include "vision_config.h"
#include "util.h"
#include "odometry.h"
#include "player.h"
#include "goalie.h"
#include "control.h"

void opcontrol() {
	initialize_robot();

	// generate_path();
	// driver();
	// shootout_goalie();
	shootout_player();
}

void driver() {
	pros::Task drive_task(driver_control);
	pros::Task odom_task(odometry);
}

void shootout_goalie() {
	while (true) {
		track_ball();
		pros::delay(10.0);
	}
}

void shootout_player() {
	follow_path();
}
