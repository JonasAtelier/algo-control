/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "imp.h"

#include <cassert>
#include <cmath>

int main()
{
	imp s = {};
	double f = 10.0;

	s.n = 1;
	s.axis[0].m = 1.0;
	s.axis[0].k = 100.0;
	s.axis[0].d = imp_damping(1.0, 100.0, 1.0);

	assert(imp_admittance(&s, &f, 0.001) == IMP_OK);
	assert(s.offset[0] > 0.0);
	imp_reset(&s);
	assert(s.offset[0] == 0.0);
	return 0;
}
