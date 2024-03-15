#include "path.h"

#include "control.h"

#include <fstream>
#include <chrono>
#include <vector>

int last_found_idx = 0;

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
            pros::delay(10.0);
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

void set_points_curvature() {
    for (int i = 1; i < POINTS-1; i++) {
        Point prev_point = points[i-1], cur_point = points[i], next_point = points[i+1];
        double dist_1 = point_to_point_dist(cur_point.x, cur_point.y, prev_point.x, prev_point.y);
        double dist_2 = point_to_point_dist(cur_point.x, cur_point.y, next_point.x, next_point.y);
        double dist_3 = point_to_point_dist(prev_point.x, prev_point.y, next_point.x, next_point.y);
        
        double product_of_sides = dist_1 * dist_2 * dist_3;
        double semi_perimeter = (dist_1 + dist_2 + dist_3) / 2.0;
        double triangle_area = sqrt(semi_perimeter * (semi_perimeter - dist_1) * (semi_perimeter - dist_2) * (semi_perimeter - dist_3));
        
        double r = product_of_sides / (4 * triangle_area);
        double curvature = isnan(1 / r) ? 0 : 1/r;
        points[i].curvature = curvature;
    }
    
    points[0].curvature = 0;
    points[POINTS-1].curvature = 0;
}

Point get_point(double x, double y, double look_ahead_dist) {
    if (last_found_idx >= POINTS-2 && point_to_point_dist(points[POINTS-1].x, points[POINTS-1].y, x, y) < look_ahead_dist)
        return points[POINTS-1];

    for (int i = last_found_idx; i < POINTS-1; i++) {
        double line_x1 = points[i].x - x, line_y1 = points[i].y - y;
        double line_x2 = points[i+1].x - x, line_y2 = points[i+1].y - y;

        double dx = line_x2 - line_x1, dy = line_y2 - line_y1;
        double dr = sqrt(pow(dx, 2) + pow(dy, 2));
        double D = line_x1 * line_y2 - line_x2 * line_y1;
        double discriminant = pow(look_ahead_dist, 2) * pow(dr, 2) - pow(D, 2);

        if (discriminant < 0)
            return points[last_found_idx];
    
        double sol1_x = (D * dy + sign(dy) * dx * sqrt(discriminant)) / pow(dr, 2) + x;
        double sol1_y = (-D * dx + abs(dy) * sqrt(discriminant)) / pow(dr, 2) + y;
        double sol2_x = (D * dy - sign(dy) * dx * sqrt(discriminant)) / pow(dr, 2) + x;
        double sol2_y = (-D * dx - abs(dy) * sqrt(discriminant)) / pow(dr, 2) + y;

        bool sol1_valid = valid_point(sol1_x, sol1_y, points[i].x, points[i].y, points[i+1].x, points[i+1].y);
        bool sold2_valid = valid_point(sol2_x, sol2_y, points[i].x, points[i].y, points[i+1].x, points[i+1].y);

        if (!(sol1_valid || sold2_valid)) {
            double dist1 = point_to_point_dist(x, y, points[i].x, points[i].y);
            double dist2 = point_to_point_dist(x, y, points[i+1].x, points[i+1].y);

            if (std::min(dist1, dist2) > look_ahead_dist) {
                return points[last_found_idx];
            } else {
                continue;
            }
        }

        Point sol1 = {sol1_x, sol1_y}, sol2 = {sol2_x, sol2_y};
        
        Point *goal_point = nullptr;
        if (!sol1_valid || !sold2_valid)
            goal_point = sol1_valid ? &sol1 : &sol2;

        if (sol1_valid && sold2_valid) {
            double firstPointDist = point_to_point_dist(sol1_x, sol1_y, points[i+1].x, points[i+1].y);
            double secondPointdist = point_to_point_dist(sol2_x, sol2_y, points[i+1].x, points[i+1].y);
            
            goal_point = firstPointDist < secondPointdist ? &sol1 : &sol2;
        }
        
        double goal_point_dist = point_to_point_dist(goal_point->x, goal_point->y, points[i+1].x, points[i+1].y);
        double current_dist = point_to_point_dist(x, y, points[i+1].x, points[i+1].y);
        
        if (current_dist > goal_point_dist) {
            last_found_idx = i;
            double curvature = (points[i].curvature + points[i+1].curvature) / 2.0;
            goal_point->curvature = curvature;
            return *goal_point;
        }
        else
            last_found_idx = i+1;
    }
    
    return points[last_found_idx];
}

bool valid_point(double point_x, double point_y, double x1, double y1, double x2, double y2) {
    bool x_valid = std::min(x1, x2) <= point_x && point_x <= std::max(x1, x2);
    bool y_valid = std::min(y1, y2) <= point_y && point_y <= std::max(y1, y2);
    
    x_valid |= abs(x1 - point_x) + abs(point_x - x2) < 0.0001;
    y_valid |= abs(y1 - point_y) + abs(point_y - y2) < 0.0001;
    
    return x_valid && y_valid;
}

double point_to_point_dist(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}
