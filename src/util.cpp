#include "util.h"

double mod_angle(double a) {
	a = fmod(fmod(a, M_PI*2)+M_PI*2, M_PI*2);
	if (a > M_PI) a -= M_PI*2;
	return a;
}

int sign(int a) {
	if (a < 0) return -1;
	else return a != 0;
}

int sign(double a) {
	if (a < 0.0) return -1;
	else return fabs(a - 0.0) > 0.000001;
}
