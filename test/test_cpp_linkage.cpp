/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "lqr.h"

#include <cassert>
#include <cmath>

int main()
{
	/* scalar system, closed form: P is the golden ratio */
	const double A = 1.0, B = 1.0, Q = 1.0, R = 1.0;
	double K = 0.0, P = 0.0;
	const double phi = (1.0 + std::sqrt(5.0)) / 2.0;

	assert(lqr_solve(1, 1, &A, &B, &Q, &R, &K, &P, NULL) == LQR_OK);
	assert(std::fabs(P - phi) < 1e-9);

	assert(lqr_gain(1, 1, &A, &B, &Q, &R, &K, NULL) == LQR_OK);

	double x = 1.0, xn = 0.0;
	lqr_step(1, 1, &A, &B, &K, &x, &xn);
	assert(std::fabs(xn) < 1.0);		/* the loop is stable, so it shrank */
	return 0;
}
