/* The header claims C++ can use it. Prove the extern "C" block works, for
 * both layers - ESP-IDF applications are routinely C++, and a missing guard
 * shows up as link errors people blame on their own build. */
#include "foc.h"
#include "upid.h"

#include <cassert>
#include <cmath>

int main()
{
	const struct foc_abc i = { 1.0f, -0.5f, -0.5f };
	const struct foc_dq idle = { 0.0f, 0.0f };
	struct foc_abc duty;
	struct foc_ab v;
	struct foc_dq dq;

	v = foc_clarke(i);
	dq = foc_park(v, 0.0f);
	assert(std::fabs(v.al - 1.0f) < 1e-4f);		/* amplitude-invariant */
	assert(std::fabs(dq.d - 1.0f) < 1e-4f);
	assert(foc_svm(v, 24.0f, &duty) == FOC_OK);
	assert(duty.a >= 0.0f && duty.a <= 1.0f);

	struct upid_cfg pcfg;
	struct upid pid_d, pid_q;
	struct foc_cfg cfg;
	struct foc_ctrl c;

	upid_cfg_init(&pcfg);
	assert(upid_create(&pid_d, &pcfg) == UPID_OK);
	assert(upid_create(&pid_q, &pcfg) == UPID_OK);
	assert(foc_cfg_init(&cfg) == FOC_OK);
	assert(foc_create(&c, &cfg, &pid_d, &pid_q) == FOC_OK);
	assert(foc_spin(&c, idle, i, 0.0f, 1e-4f, &duty) == FOC_OK);
	return 0;
}
