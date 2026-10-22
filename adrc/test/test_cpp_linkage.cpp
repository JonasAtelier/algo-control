/* The header claims C++ can use it. Prove the extern "C" block works. */
#include "adrc.h"

#include <cassert>
#include <cmath>

int main()
{
	adrc_cfg cfg = { 1, 2.0, 20.0, 0.0, 0.0, 0.0 };	/* wo = 0: pick it for me */
	adrc s;

	assert(adrc_init(&s, &cfg) == ADRC_OK);
	assert(std::fabs(s.cfg.wo - 100.0) < 1e-12);	/* 5 x wc */

	adrc_reset(&s, 0.0);
	assert(std::isfinite(adrc_update(&s, 1.0, 0.0, 1e-3)));
	return 0;
}
