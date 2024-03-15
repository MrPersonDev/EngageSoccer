#include "path.h"

#include "control.h"

#include <fstream>
#include <chrono>

void generate_path() {
	const double sample = 500.0;
	std::ofstream ofs{"/usd/path.text", std::ofstream::trunc};
    ofs << "Point path[] = {{0,0}";

    int points = 1;
	while (!cancel_input()) {
		using namespace std::chrono;
		auto epoch = high_resolution_clock::now();
        while (duration_cast<milliseconds>(high_resolution_clock::now() - epoch).count() <= sample) {
            holonomic_drive();
            update_position();
        }
		ofs << ", {" << get_robot_x() << "," << get_robot_y() << "}";
		points++;
    }
    ofs << "};\n";

    ofs << "const int POINTS = " << points << ";\n";
    ofs << "const int CURVES = " << points - 1 << ";\n";
	ofs.flush();
    ofs.close();

	controller.rumble("...");
}

Point get_point(double t) {
    double time_relative = t/SAMPLING; // account for special relativity

    int index = (int)(time_relative);
    if(index >= CURVES) { // last point would go out of bounds
        index = CURVES - 1;
        t = 1;
    } else {
        t = time_relative - index; // since bezier curves are 0 <= t <= 1 we need to shift the time from the total to the local curve
	}
        
    double x = (1-t)*(1-t)*(1-t)*position[index].p0.x + 3*t*(1-t)*(1-t)*position[index].p1.x + 3*t*t*(t-1)*position[index].p2.x + t*t*t*position[index].p3.x;
    double y = (1-t)*(1-t)*(1-t)*position[index].p0.y + 3*t*(1-t)*(1-t)*position[index].p1.y + 3*t*t*(t-1)*position[index].p2.y + t*t*t*position[index].p3.y;
    
    return {x, y};
}

double get_heading(double t) {
	double time_relative = t/SAMPLING;

    int index = (int)(time_relative);
    if(index >= CURVES) {
        index = CURVES - 1;
        t = 1;
    }
    else {
        t = time_relative - index; // since bezier curves are 0 <= t <= 1 we need to shift the time from the total to the local curve
	}
    
	// using the derivative of a bezier curve
    double vx = -3*(1-t)*(1-t)*position[index].p0.x + (3-12*t+9*t*t)*position[index].p1.x + (6*t-9*t*t)*position[index].p2.x + 3*t*t*position[index].p3.x;
    double vy = -3*(1-t)*(1-t)*position[index].p0.y + (3-12*t+9*t*t)*position[index].p1.y + (6*t-9*t*t)*position[index].p2.y + 3*t*t*position[index].p3.y;
    
    return atan2(vy, vx);
}
