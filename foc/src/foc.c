/*
 * foc - the transforms and the modulator.
 *
 * Pure functions over three floats. Nothing here reads a clock, keeps state,
 * allocates, or knows that upid exists. This file compiles on its own with
 * nothing but libm, which is the point: take it without the current loop if
 * that is all you want.
 */
#include "foc.h"

#include <math.h>
#include <stddef.h>

#define FOC_PI		3.14159265358979323846f
#define FOC_2PI		6.28318530717958647692f
#define FOC_SQRT3	1.73205080756887729353f
#define FOC_SQRT3_2	0.86602540378443864676f	/* sqrt(3)/2 */

static float clamp01(float x)
{
	if (x < 0.0f)
		return 0.0f;
	if (x > 1.0f)
		return 1.0f;
	return x;
}

struct foc_ab foc_clarke(struct foc_abc x)
{
	struct foc_ab r;

	/* Amplitude-invariant: the 2/3 is what makes a 10 A peak phase
	 * current read as 10 on the axes. See the header before comparing
	 * these numbers against a textbook. */
	r.al = (2.0f * x.a - x.b - x.c) / 3.0f;
	r.be = (x.b - x.c) / FOC_SQRT3;
	return r;
}

struct foc_abc foc_clarke_inv(struct foc_ab x)
{
	struct foc_abc r;

	/* Sums to zero by construction, because there is no third wire for a
	 * zero-sequence current to return through. */
	r.a = x.al;
	r.b = -0.5f * x.al + FOC_SQRT3_2 * x.be;
	r.c = -0.5f * x.al - FOC_SQRT3_2 * x.be;
	return r;
}

struct foc_dq foc_park(struct foc_ab x, float theta_e)
{
	float s = sinf(theta_e);
	float c = cosf(theta_e);
	struct foc_dq r;

	r.d = x.al * c + x.be * s;
	r.q = -x.al * s + x.be * c;
	return r;
}

struct foc_ab foc_park_inv(struct foc_dq x, float theta_e)
{
	float s = sinf(theta_e);
	float c = cosf(theta_e);
	struct foc_ab r;

	r.al = x.d * c - x.q * s;
	r.be = x.d * s + x.q * c;
	return r;
}

float foc_wrap(float rad)
{
	/* fmodf rather than a subtract-until-in-range loop: one call, and it
	 * still terminates when a mis-scaled encoder hands you a thousand
	 * radians. fmodf keeps the sign of its left argument, hence the
	 * second adjustment. */
	float w = fmodf(rad + FOC_PI, FOC_2PI);

	if (w < 0.0f)
		w += FOC_2PI;
	return w - FOC_PI;
}

float foc_v_max(float v_dc)
{
	return v_dc / FOC_SQRT3;
}

foc_sta foc_svm(struct foc_ab v, float v_dc, struct foc_abc *duty)
{
	foc_sta sta = FOC_OK;
	struct foc_abc p;
	float mag, lim, hi, lo, mid;

	if (duty == NULL || v_dc <= 0.0f ||
	    !isfinite(v.al) || !isfinite(v.be) || !isfinite(v_dc))
		return FOC_EINVAL;

	lim = foc_v_max(v_dc);
	mag = sqrtf(v.al * v.al + v.be * v.be);
	if (mag > lim) {
		/* Scale the vector, do not clip the phases.
		 *
		 * This will look like the long way round to a future reader.
		 * It is not. Clipping each duty at 0 and 1 shortens whichever
		 * phase is furthest out by more than the other two, and a
		 * three-phase set shortened unevenly is a vector pointing
		 * somewhere else - so the torque angle moves away from the one
		 * the current loop just worked out, and the motor pulls in a
		 * direction nobody asked for. Scaling loses magnitude and
		 * keeps the direction, which is the error you can live with:
		 * less torque, still the right torque. */
		float k = lim / mag;

		v.al *= k;
		v.be *= k;
		sta = FOC_ESAT;
	}

	p = foc_clarke_inv(v);

	/* Min-max midpoint injection. Shifting all three phases by the same
	 * amount cancels in every phase-to-phase voltage, so the motor cannot
	 * tell - but it recentres the trio in [0, 1] and buys 2/sqrt(3) of
	 * headroom. Identical duties to sector-based SVPWM, without the six
	 * sectors and their boundaries. */
	hi = p.a > p.b ? p.a : p.b;
	if (p.c > hi)
		hi = p.c;
	lo = p.a < p.b ? p.a : p.b;
	if (p.c < lo)
		lo = p.c;
	mid = 0.5f * (hi + lo);

	/* The clamps only mop up float error at the exact limit; the scaling
	 * above is what keeps the duties in range. */
	duty->a = clamp01(0.5f + (p.a - mid) / v_dc);
	duty->b = clamp01(0.5f + (p.b - mid) / v_dc);
	duty->c = clamp01(0.5f + (p.c - mid) / v_dc);

	return sta;
}
