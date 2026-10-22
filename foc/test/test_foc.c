/* Self-check: assert-based, no framework. Build with `make check`.
 *
 * This is a maths library, so these tests are the product. They check the
 * identities the transforms are supposed to satisfy rather than the numbers
 * one particular implementation happens to produce.
 */
#include "foc.h"
#include "upid.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f

static int close_to(float a, float b, float tol)
{
	return fabsf(a - b) <= tol;
}

/* A balanced three-phase set of peak amplitude amp at electrical angle th. */
static struct foc_abc balanced(float amp, float th)
{
	struct foc_abc x;

	x.a = amp * cosf(th);
	x.b = amp * cosf(th - 2.0f * PI / 3.0f);
	x.c = amp * cosf(th + 2.0f * PI / 3.0f);
	return x;
}

/*
 * ---------------------------------------------------------------------------
 * The transforms
 * ---------------------------------------------------------------------------
 */

static void test_clarke_round_trips(void)
{
	int i;

	/* Only balanced sets survive the round trip, because Clarke discards
	 * the zero sequence a three-wire motor cannot carry anyway. */
	for (i = 0; i < 32; i++) {
		float th = -PI + i * (2.0f * PI / 32.0f);
		struct foc_abc in = balanced(7.5f, th);
		struct foc_abc out = foc_clarke_inv(foc_clarke(in));

		assert(close_to(out.a, in.a, 1e-4f));
		assert(close_to(out.b, in.b, 1e-4f));
		assert(close_to(out.c, in.c, 1e-4f));
	}
}

static void test_clarke_discards_zero_sequence(void)
{
	struct foc_abc in = { 1.0f, 1.0f, 1.0f };
	struct foc_ab out = foc_clarke(in);
	struct foc_abc back = foc_clarke_inv(out);

	/* A pure common-mode set is invisible in the stationary frame. */
	assert(close_to(out.al, 0.0f, 1e-6f));
	assert(close_to(out.be, 0.0f, 1e-6f));
	/* And the inverse always hands back a set summing to zero. */
	assert(close_to(back.a + back.b + back.c, 0.0f, 1e-6f));
}

static void test_clarke_is_amplitude_invariant(void)
{
	/* 10 A peak in the phases must read as 10 on the axes, not 12.25.
	 * This pins the 2/3 convention against the power-invariant one. */
	struct foc_ab v = foc_clarke(balanced(10.0f, 0.0f));

	assert(close_to(v.al, 10.0f, 1e-4f));
	assert(close_to(v.be, 0.0f, 1e-4f));
}

static void test_park_round_trips(void)
{
	int i;

	for (i = 0; i < 32; i++) {
		float th = -PI + i * (2.0f * PI / 32.0f);
		struct foc_ab in;
		struct foc_ab out;

		in.al = 3.0f;
		in.be = -1.25f;
		out = foc_park_inv(foc_park(in, th), th);

		assert(close_to(out.al, in.al, 1e-4f));
		assert(close_to(out.be, in.be, 1e-4f));
	}
}

static void test_balanced_set_has_constant_magnitude(void)
{
	int i;

	/* Three sinusoids 120 degrees apart trace a circle, not an ellipse.
	 * If they did not, the whole scheme would produce torque ripple. */
	for (i = 0; i < 64; i++) {
		float th = -PI + i * (2.0f * PI / 64.0f);
		struct foc_ab v = foc_clarke(balanced(4.0f, th));
		float mag = sqrtf(v.al * v.al + v.be * v.be);

		assert(close_to(mag, 4.0f, 1e-4f));
	}
}

static void test_park_turns_rotation_into_constants(void)
{
	int i;

	/* THE test. Field-oriented control is exactly this claim: a current
	 * vector spinning with the rotor, viewed from the rotor, is two
	 * numbers that do not move - so a plain PID can regulate them. */
	for (i = 0; i < 64; i++) {
		float th = -PI + i * (2.0f * PI / 64.0f);
		struct foc_dq x = foc_park(foc_clarke(balanced(6.0f, th)), th);

		assert(close_to(x.d, 6.0f, 1e-3f));
		assert(close_to(x.q, 0.0f, 1e-3f));
	}
}

static void test_park_at_quarter_turn_is_all_q(void)
{
	/* A vector a quarter electrical turn ahead of the rotor is pure
	 * torque. Catches a sign flip that the round-trip test cannot see,
	 * because a round trip is happy with either sign convention. */
	struct foc_dq x = foc_park(foc_clarke(balanced(6.0f, 0.0f)), -PI / 2.0f);

	assert(close_to(x.d, 0.0f, 1e-3f));
	assert(close_to(x.q, 6.0f, 1e-3f));
}

/*
 * ---------------------------------------------------------------------------
 * Angle wrapping
 * ---------------------------------------------------------------------------
 */

static void test_wrap_keeps_the_angle_it_was_given(void)
{
	static const float in[] = {
		0.0f, 0.1f, -0.1f, 3.0f, -3.0f,
		PI, -PI, 2.0f * PI + 0.1f, -2.0f * PI - 0.1f,
		9.0f * PI, -9.0f * PI, 1000.0f
	};
	unsigned i;

	for (i = 0; i < sizeof in / sizeof in[0]; i++) {
		float w = foc_wrap(in[i]);

		/* in range ... */
		assert(w >= -PI - 1e-4f && w <= PI + 1e-4f);
		/* ... and still the same direction */
		assert(close_to(cosf(w), cosf(in[i]), 1e-3f));
		assert(close_to(sinf(w), sinf(in[i]), 1e-3f));
	}
}

static void test_wrap_across_the_boundary(void)
{
	/* The reason wrapping exists: two angles either side of +/-pi are
	 * close together, and the naive subtraction says they are a whole
	 * turn apart. */
	float diff = foc_wrap(3.13f - (-3.13f));

	assert(close_to(diff, 6.26f - 2.0f * PI, 1e-4f));
	assert(fabsf(diff) < 0.05f);

	assert(close_to(foc_wrap(0.1f), 0.1f, 1e-6f));
	assert(close_to(foc_wrap(2.0f * PI + 0.1f), 0.1f, 1e-4f));
	assert(close_to(foc_wrap(-2.0f * PI - 0.1f), -0.1f, 1e-4f));
}

/*
 * ---------------------------------------------------------------------------
 * Modulation
 * ---------------------------------------------------------------------------
 */

static void test_duties_stay_in_range(void)
{
	const float v_dc = 24.0f;
	int i, j;

	/* Including amplitudes well past what the bus can do: a duty outside
	 * [0, 1] is not a number a PWM peripheral can be handed. */
	for (i = 1; i <= 40; i++) {
		float amp = i * 1.0f;

		for (j = 0; j < 180; j++) {
			float th = -PI + j * (2.0f * PI / 180.0f);
			struct foc_ab v;
			struct foc_abc d;
			foc_sta sta;

			v.al = amp * cosf(th);
			v.be = amp * sinf(th);
			sta = foc_svm(v, v_dc, &d);

			assert(sta == FOC_OK || sta == FOC_ESAT);
			assert(d.a >= 0.0f && d.a <= 1.0f);
			assert(d.b >= 0.0f && d.b <= 1.0f);
			assert(d.c >= 0.0f && d.c <= 1.0f);
		}
	}
}

static void test_duties_reproduce_the_requested_vector(void)
{
	const float v_dc = 24.0f;
	/* 9 V is inside what plain sine modulation could manage. 13 V is not:
	 * it is past v_dc/2 and only reachable because of the injection. */
	static const float amps[] = { 9.0f, 13.0f };
	unsigned n;
	int j;

	for (n = 0; n < sizeof amps / sizeof amps[0]; n++) {
		for (j = 0; j < 90; j++) {
			float th = -PI + j * (2.0f * PI / 90.0f);
			struct foc_ab v, back;
			struct foc_abc d, want;

			v.al = amps[n] * cosf(th);
			v.be = amps[n] * sinf(th);
			assert(foc_svm(v, v_dc, &d) == FOC_OK);

			/* Clarke of the duties throws away the injected common
			 * mode, which is why injecting it is free. */
			back = foc_clarke(d);
			assert(close_to(back.al * v_dc, v.al, 2e-3f));
			assert(close_to(back.be * v_dc, v.be, 2e-3f));

			/* And the phase-to-phase voltages the motor sees. */
			want = foc_clarke_inv(v);
			assert(close_to((d.a - d.b) * v_dc, want.a - want.b, 2e-3f));
			assert(close_to((d.b - d.c) * v_dc, want.b - want.c, 2e-3f));
			assert(close_to((d.c - d.a) * v_dc, want.c - want.a, 2e-3f));
		}
	}
}

static void test_saturation_keeps_the_angle(void)
{
	const float v_dc = 24.0f;
	struct foc_ab v, back;
	struct foc_abc d;

	/* Asking for four times the bus. Per-phase clipping would shorten one
	 * phase more than the others and swing the vector round, which moves
	 * the torque angle. Scaling must leave the direction alone. */
	v.al = 40.0f * cosf(1.1f);
	v.be = 40.0f * sinf(1.1f);
	assert(foc_svm(v, v_dc, &d) == FOC_ESAT);

	back = foc_clarke(d);
	assert(close_to(atan2f(back.be, back.al), 1.1f, 1e-3f));
	assert(close_to(sqrtf(back.al * back.al + back.be * back.be) * v_dc,
			foc_v_max(v_dc), 1e-3f));
}

static void test_injection_beats_sine_by_two_over_root_three(void)
{
	const float v_dc = 24.0f;
	float sine_ceiling = 0.0f, svm_ceiling = 0.0f;
	int i, j;

	/* The whole reason min-max injection exists. Plain sine modulation
	 * runs out at v_dc/2, because a duty of 0.5 + v/v_dc leaves [0, 1]
	 * there. Subtracting the min-max midpoint buys 2/sqrt(3), about
	 * 15.5%, and nothing else about the output changes. Drop the
	 * injection term and this is the test that notices.
	 *
	 * "Linear range" means the vector comes back out at EVERY angle, not
	 * at the best one. Checking only the largest reconstruction over a
	 * revolution would pass a modulator that clips near the phase peaks,
	 * because the angles between them do not clip. */
	for (i = 10; i <= 2400; i++) {
		float amp = i * 0.01f;
		int sine_fits = 1;
		int svm_fits = 1;

		for (j = 0; j < 180; j++) {
			float th = -PI + j * (2.0f * PI / 180.0f);
			struct foc_ab v, back;
			struct foc_abc d, p;

			v.al = amp * cosf(th);
			v.be = amp * sinf(th);

			/* sine modulation, spelled out here so the comparison
			 * does not lean on the library it is judging */
			p = foc_clarke_inv(v);
			if (0.5f + p.a / v_dc > 1.0f || 0.5f + p.a / v_dc < 0.0f ||
			    0.5f + p.b / v_dc > 1.0f || 0.5f + p.b / v_dc < 0.0f ||
			    0.5f + p.c / v_dc > 1.0f || 0.5f + p.c / v_dc < 0.0f)
				sine_fits = 0;

			foc_svm(v, v_dc, &d);
			assert(d.a >= 0.0f && d.a <= 1.0f);
			assert(d.b >= 0.0f && d.b <= 1.0f);
			assert(d.c >= 0.0f && d.c <= 1.0f);

			/* what the motor got, not what we asked for */
			back = foc_clarke(d);
			if (!close_to(back.al * v_dc, v.al, 3e-3f) ||
			    !close_to(back.be * v_dc, v.be, 3e-3f))
				svm_fits = 0;
		}
		if (sine_fits)
			sine_ceiling = amp;
		if (svm_fits)
			svm_ceiling = amp;
	}

	assert(sine_ceiling > 0.0f);
	assert(close_to(sine_ceiling, v_dc / 2.0f, 0.02f));
	assert(close_to(svm_ceiling, foc_v_max(v_dc), 0.02f));
	assert(close_to(svm_ceiling / sine_ceiling, 2.0f / sqrtf(3.0f), 0.01f));
}

/*
 * ---------------------------------------------------------------------------
 * Bad arguments
 * ---------------------------------------------------------------------------
 */

static void test_bad_args(void)
{
	struct foc_cfg cfg;
	struct foc_ctrl c;
	struct foc_abc d, zero = { 0.0f, 0.0f, 0.0f };
	struct foc_dq tgt = { 0.0f, 0.0f };
	struct foc_ab v = { 1.0f, 1.0f };
	struct upid pid_d, pid_q;
	struct upid_cfg pcfg;

	assert(foc_svm(v, 24.0f, NULL) == FOC_EINVAL);
	assert(foc_svm(v, 0.0f, &d) == FOC_EINVAL);
	assert(foc_svm(v, -1.0f, &d) == FOC_EINVAL);
	v.al = NAN;
	assert(foc_svm(v, 24.0f, &d) == FOC_EINVAL);

	assert(foc_cfg_init(NULL) == FOC_EINVAL);
	assert(foc_cfg_init(&cfg) == FOC_OK);

	upid_cfg_init(&pcfg);
	assert(upid_create(&pid_d, &pcfg) == UPID_OK);
	assert(upid_create(&pid_q, &pcfg) == UPID_OK);

	assert(foc_create(NULL, &cfg, &pid_d, &pid_q) == FOC_EINVAL);
	assert(foc_create(&c, NULL, &pid_d, &pid_q) == FOC_EINVAL);
	assert(foc_create(&c, &cfg, NULL, &pid_q) == FOC_EINVAL);
	assert(foc_create(&c, &cfg, &pid_d, NULL) == FOC_EINVAL);

	cfg.pole_pairs = 0;
	assert(foc_create(&c, &cfg, &pid_d, &pid_q) == FOC_EINVAL);
	cfg.pole_pairs = 7;
	cfg.v_dc = 0.0f;
	assert(foc_create(&c, &cfg, &pid_d, &pid_q) == FOC_EINVAL);
	cfg.v_dc = 24.0f;

	/* A controller create() refused must stay refused, not run on the
	 * half-applied config. */
	assert(foc_spin(&c, tgt, zero, 0.0f, 1e-4f, &d) == FOC_EINVAL);

	assert(foc_create(&c, &cfg, &pid_d, &pid_q) == FOC_OK);
	assert(foc_spin(&c, tgt, zero, 0.0f, 0.0f, &d) == FOC_EINVAL);
	assert(foc_spin(&c, tgt, zero, 0.0f, 1e-4f, NULL) == FOC_EINVAL);
	assert(foc_spin(NULL, tgt, zero, 0.0f, 1e-4f, &d) == FOC_EINVAL);
	zero.a = NAN;
	assert(foc_spin(&c, tgt, zero, 0.0f, 1e-4f, &d) == FOC_EINVAL);
}

static void test_electrical_angle(void)
{
	struct foc_cfg cfg;

	assert(foc_cfg_init(&cfg) == FOC_OK);
	cfg.pole_pairs = 7;
	cfg.theta_offset = 0.0f;
	cfg.reverse = false;

	/* Seven electrical turns per mechanical turn, wrapped. */
	assert(close_to(foc_wrap(foc_elec_angle(&cfg, 0.1f) - 0.7f), 0.0f, 1e-4f));
	assert(foc_elec_angle(&cfg, 1.0f) >= -PI &&
	       foc_elec_angle(&cfg, 1.0f) <= PI);

	cfg.reverse = true;
	assert(close_to(foc_wrap(foc_elec_angle(&cfg, 0.1f) + 0.7f), 0.0f, 1e-4f));

	cfg.reverse = false;
	cfg.theta_offset = 0.7f;
	assert(close_to(foc_elec_angle(&cfg, 0.1f), 0.0f, 1e-4f));
}

/*
 * ---------------------------------------------------------------------------
 * The current loop, against a simulated motor
 * ---------------------------------------------------------------------------
 *
 * One phase of an ideal PMSM held at standstill: L di/dt = v - R i, no
 * back-EMF, no saliency, no cross-coupling. That is enough to tell a working
 * current loop from a broken one, and every simplification here makes the
 * wrong-offset failure below harder to produce, not easier.
 */

#define SIM_R	1.0f		/* phase resistance, ohms */
#define SIM_L	0.001f		/* phase inductance, henries */
#define SIM_DT	100e-6f		/* 10 kHz control loop */
#define SIM_VDC	24.0f
#define SIM_PP	7
#define SIM_TM	0.3f		/* mechanical angle the rotor is clamped at */

/*
 * Run the loop for a while and report the current in the TRUE rotor frame -
 * the frame the magnets are actually in, which the controller only knows
 * about if theta_offset is right.
 */
static struct foc_dq run_loop(float offset_used, float target_q, int steps)
{
	const float theta_true = SIM_PP * SIM_TM;	/* true offset is zero */
	struct foc_dq i_true = { 0.0f, 0.0f };
	struct upid pid_d, pid_q;
	struct upid_cfg pcfg;
	struct foc_cfg cfg;
	struct foc_ctrl c;
	int k;

	/* Textbook current-loop tuning: kp = L w, ki = R w. */
	upid_cfg_init(&pcfg);
	pcfg.kp = SIM_L * 2000.0f;
	pcfg.ki = SIM_R * 2000.0f;
	pcfg.o_min = -foc_v_max(SIM_VDC);
	pcfg.o_max = foc_v_max(SIM_VDC);
	assert(upid_create(&pid_d, &pcfg) == UPID_OK);
	assert(upid_create(&pid_q, &pcfg) == UPID_OK);

	assert(foc_cfg_init(&cfg) == FOC_OK);
	cfg.pole_pairs = SIM_PP;
	cfg.v_dc = SIM_VDC;
	cfg.theta_offset = offset_used;
	assert(foc_create(&c, &cfg, &pid_d, &pid_q) == FOC_OK);

	for (k = 0; k < steps; k++) {
		struct foc_dq tgt = { 0.0f, 0.0f };
		struct foc_abc duty, i_abc;
		struct foc_ab v_ab;
		struct foc_dq v_true;
		foc_sta sta;

		tgt.q = target_q;

		/* what the current sensors would report */
		i_abc = foc_clarke_inv(foc_park_inv(i_true, theta_true));

		sta = foc_spin(&c, tgt, i_abc, SIM_TM, SIM_DT, &duty);
		assert(sta == FOC_OK || sta == FOC_ESAT);

		/* what the bridge actually applies, read back off the duties */
		v_ab = foc_clarke(duty);
		v_ab.al *= SIM_VDC;
		v_ab.be *= SIM_VDC;
		v_true = foc_park(v_ab, theta_true);

		/* L di/dt = v - R i, forward Euler */
		i_true.d += (v_true.d - SIM_R * i_true.d) * SIM_DT / SIM_L;
		i_true.q += (v_true.q - SIM_R * i_true.q) * SIM_DT / SIM_L;
	}
	return i_true;
}

static void test_loop_puts_the_current_where_it_was_asked(void)
{
	struct foc_dq i = run_loop(0.0f, 2.0f, 1000);

	/* All 2 A on the torque axis, none on the flux axis. */
	assert(close_to(i.q, 2.0f, 0.02f));
	assert(close_to(i.d, 0.0f, 0.02f));
}

static void test_wrong_offset_draws_current_and_makes_no_torque(void)
{
	struct foc_dq good = run_loop(0.0f, 2.0f, 1000);
	struct foc_dq bad = run_loop(PI / 2.0f, 2.0f, 1000);
	float good_mag = sqrtf(good.d * good.d + good.q * good.q);
	float bad_mag = sqrtf(bad.d * bad.d + bad.q * bad.q);

	/* This is the calibration knob earning its place. A quarter turn of
	 * error and the motor draws exactly the same current - the loop still
	 * reports perfect tracking, because in the frame it believes in the
	 * tracking IS perfect - but every amp of it now makes flux instead of
	 * torque. The motor gets hot and does not turn, and nothing in
	 * software can tell you why. */
	assert(close_to(bad_mag, good_mag, 0.05f));	/* same amps drawn */
	assert(fabsf(bad.q) < 0.05f);			/* no torque */
	assert(close_to(fabsf(bad.d), 2.0f, 0.05f));	/* all of it flux */
}

int main(void)
{
	test_clarke_round_trips();
	test_clarke_discards_zero_sequence();
	test_clarke_is_amplitude_invariant();
	test_park_round_trips();
	test_balanced_set_has_constant_magnitude();
	test_park_turns_rotation_into_constants();
	test_park_at_quarter_turn_is_all_q();
	test_wrap_keeps_the_angle_it_was_given();
	test_wrap_across_the_boundary();
	test_duties_stay_in_range();
	test_duties_reproduce_the_requested_vector();
	test_saturation_keeps_the_angle();
	test_injection_beats_sine_by_two_over_root_three();
	test_bad_args();
	test_electrical_angle();
	test_loop_puts_the_current_where_it_was_asked();
	test_wrong_offset_draws_current_and_makes_no_torque();
	printf("all tests passed\n");
	return 0;
}
