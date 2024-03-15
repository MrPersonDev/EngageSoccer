#include "util.h"

double mod_angle(double a) {
	a = fmod(fmod(a, M_PI*2)+M_PI*2, M_PI*2);
	if (a > M_PI) a -= M_PI*2;
	return a;
}
