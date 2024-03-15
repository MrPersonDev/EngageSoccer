#include "chassis_config.h"

void initialize_robot() {
	vision_sensor.set_signature(1, &Vision2__SIG_1);
	vision_sensor.set_signature(1, &Vision2__SIG_2);
	vision_sensor.set_signature(1, &Vision2__SIG_3);
	vision_sensor.set_signature(1, &Vision2__SIG_4);
	pros::lcd::initialize();
	pl_encoder.reset();
	pd_encoder.reset();
	inertial_sensor.reset();
	while (inertial_sensor.is_calibrating()) {
		pros::delay(20.0);
	}
}
