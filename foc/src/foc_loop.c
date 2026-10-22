/*
 * foc - the d-q current loop.
 *
 * The only file in this library that includes upid.h. Delete it and the
 * transforms in foc.c still build with nothing but libm.
 *
 * There is no PID wrapper here on purpose: the two controllers belong to the
 * caller, who created and tuned them, and this file only calls upid_spin() on
 * them. Anything else would mean re-exporting every knob upid already has.
 */
#include "foc.h"
#include "upid.h"

#include <math.h>
#include <stddef.h>

#define FOC_MARKER 0x464F4331u	/* "FOC1" */

foc_sta foc_cfg_init(struct foc_cfg *cfg)
{
	if (cfg == NULL)
		return FOC_EINVAL;

	cfg->pole_pairs = 7;
	cfg->theta_offset = 0.0f;
	cfg->reverse = false;
	cfg->v_dc = 12.0f;
	return FOC_OK;
}

static foc_sta cfg_check(const struct foc_cfg *cfg)
{
	if (cfg == NULL || cfg->pole_pairs < 1)
		return FOC_EINVAL;
	if (!isfinite(cfg->theta_offset))
		return FOC_EINVAL;
	if (!isfinite(cfg->v_dc) || cfg->v_dc <= 0.0f)
		return FOC_EINVAL;
	return FOC_OK;
}

foc_sta foc_create(struct foc_ctrl *c, const struct foc_cfg *cfg,
		   struct upid *pid_d, struct upid *pid_q)
{
	if (c == NULL)
		return FOC_EINVAL;

	/* Unrunnable until every check below has passed, so a controller that
	 * was refused stays refused rather than running on half a config. */
	c->marker = 0;

	if (pid_d == NULL || pid_q == NULL)
		return FOC_EINVAL;
	if (cfg_check(cfg) != FOC_OK)
		return FOC_EINVAL;
	if (upid_cfg_status(pid_d) != UPID_OK || upid_cfg_status(pid_q) != UPID_OK)
		return FOC_EINVAL;

	c->cfg = *cfg;
	c->pid_d = pid_d;
	c->pid_q = pid_q;
	c->theta_e = 0.0f;
	c->i_dq.d = 0.0f;
	c->i_dq.q = 0.0f;
	c->v_dq.d = 0.0f;
	c->v_dq.q = 0.0f;
	c->marker = FOC_MARKER;
	return FOC_OK;
}

float foc_elec_angle(const struct foc_cfg *cfg, float theta_mech)
{
	float m;

	if (cfg == NULL || !isfinite(theta_mech))
		return 0.0f;

	m = cfg->reverse ? -theta_mech : theta_mech;
	return foc_wrap((float)cfg->pole_pairs * m - cfg->theta_offset);
}

foc_sta foc_spin(struct foc_ctrl *c, struct foc_dq target, struct foc_abc i_abc,
		 float theta_mech, float dt_s, struct foc_abc *duty)
{
	struct foc_ab v_ab;

	if (c == NULL || duty == NULL || c->marker != FOC_MARKER)
		return FOC_EINVAL;
	if (!isfinite(i_abc.a) || !isfinite(i_abc.b) || !isfinite(i_abc.c))
		return FOC_EINVAL;
	if (!isfinite(target.d) || !isfinite(target.q) || !isfinite(theta_mech))
		return FOC_EINVAL;

	/* Where the rotor is, in the only frame that matters. */
	c->theta_e = foc_elec_angle(&c->cfg, theta_mech);

	/* Three moving currents become two still ones ... */
	c->i_dq = foc_park(foc_clarke(i_abc), c->theta_e);

	/* ... which is what lets a plain PID regulate them. upid validates dt
	 * and does its own anti-windup, so there is nothing to add here. */
	if (upid_spin(c->pid_d, target.d, c->i_dq.d, dt_s) != UPID_OK)
		return FOC_EINVAL;
	if (upid_spin(c->pid_q, target.q, c->i_dq.q, dt_s) != UPID_OK)
		return FOC_EINVAL;

	c->v_dq.d = upid_get_output(c->pid_d);
	c->v_dq.q = upid_get_output(c->pid_q);

	/* Back out to the stationary frame and on to three duty cycles. */
	v_ab = foc_park_inv(c->v_dq, c->theta_e);
	return foc_svm(v_ab, c->cfg.v_dc, duty);
}
