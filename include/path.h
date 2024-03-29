#ifndef PATH_H_
#define PATH_H_

#include "odometry.h"

struct Point {
	double x, y, curvature;
	bool stopping;
};

struct Bezier {
    Point p0, p1, p2, p3;
};

/* Points gathered by robot */
static Point points[] = {{0,0}, {-2.46294,2.80549}, {-4.88086,10.7583}, {-6.88698,20.5546}, {-14.1721,29.8517}, {-22.6922,38.4347}, {-31.6892,47.4833}, {-33.4478,58.1657}, {-32.2825,68.1733}, {-27.3768,73.7587}, {-21.8552,79.4778}, {-13.7681,83.1747}, {-3.90342,85.906}, {5.05333,93.148}, {10.4884,101.891}, {10.5415,112.299}, {9.35196,123.021}, {1.39132,130.191}, {-8.22969,133.939}, {-18.588,136.493}, {-29.8162,137.303}, {-41.6095,137.676}, {-53.8412,138.601}, {-63.5182,147.728}, {-67.5881,157.69}, {-68.2822,166.447}, {-68.6014,175.868}, {-62.6391,181.052}, {-53.1243,184.646}, {-42.9362,186.46}, {-32.8599,187.079}, {-22.832,186.794}, {-12.8648,186.219}, {-2.58582,185.712}, {6.76148,179.083}, {8.11367,168.609}, {8.42531,156.691}, {1.29513,146.369}, {-10.9118,140.918}, {-18.1647,136.241}, {-24.2194,130.793}, {-31.621,124.628}, {-38.976,118.265}, {-43.2873,113.244}, {-44.8871,107.147}, {-51.3679,100.885}, {-54.0551,92.8752}, {-54.305,82.4062}, {-55.8343,71.4152}, {-57.0818,60.3489}, {-57.7131,48.0423}, {-58.2268,35.6609}, {-58.4225,22.954}, {-58.6652,18.351}};
const int POINTS = 54;

void generate_path();
void set_points_curvature();
Point get_point(double x, double y, double look_ahead_dist);
bool valid_point(double point_x, double point_y, double x1, double y1, double x2, double y2);
double point_to_point_dist(double x1, double y1, double x2, double y2);

#endif // PATH_H_
