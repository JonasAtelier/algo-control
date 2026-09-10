/*
 * align.c - find theta_offset, the one number this library cannot work out
 *           for itself, and then show what happens when it is wrong.
 *
 * Everything below runs against a simulated motor, so it needs no hardware.
 * The procedure it performs is the real one, and the calls are the calls you
 * would make on a bench.
 *
 *
 * WHY THIS EXISTS
 *
 * foc knows how to put current on the rotor's d and q axes. It does not know
 * where those axes are. The rotor is a magnet glued to a shaft, the encoder
 * is a chip glued to a board, and the angle between what the encoder calls
 * zero and where the magnet's north actually points is whatever it came out
 * as on the day. No datasheet has it. No amount of software finds it. You
 * measure it once per assembled motor and store it.
 *
 *
 * WHAT YOU DO ON A REAL BENCH
 *
 *   1. Free the shaft. No propeller, no gearbox, nothing bolted on. The rotor
 *      has to be able to turn, and it is about to.
 *
 *   2. Limit the current. You are about to apply a fixed voltage to a stalled
 *      motor, which is a short circuit through the winding resistance. A few
 *      hundred millivolts across a 0.1 ohm winding is already amps. Start far
 *      lower than you think and raise it only if the rotor does not move.
 *
 *   3. Apply a voltage vector at electrical angle zero, with theta_offset set
 *      to zero, and hold it. Two lines, no current loop involved:
 *
 *          v = foc_park_inv((struct foc_dq){ v_align, 0.0f }, 0.0f);
 *          foc_svm(v, v_dc, &duty);
 *
 *      Physically: phase A is pulled high, B and C low, so the stator makes a
 *      stationary magnetic field pointing along phase A. The rotor's magnets
 *      snap round to line up with it, exactly like a compass needle finding
 *      north, and stop. That position IS electrical zero, by definition, and
 *      it is the only rotor position you can produce on demand.
 *
 *   4. Wait for it to stop moving. It will swing past and oscillate first,
 *      because there is nothing damping it but friction. A second is plenty.
 *      Nudge it gently by hand and let it settle again - if it returns to the
 *      same place, the field is holding it; if it does not, your voltage is
 *      too low or a phase is not connected.
 *
 *   5. Read the encoder and convert with the offset still at zero:
 *
 *          cfg.theta_offset = 0.0f;
 *          cfg.theta_offset = foc_elec_angle(&cfg, encoder_read_mech());
 *
 *      That is the answer. Store it in your build, or in NVS, per motor.
 *
 *
 * WHAT A GOOD RESULT LOOKS LIKE
 *
 *   - The rotor moves during step 3 and then sits still, holding position
 *     against a gentle push. If it buzzes, hums, or creeps, stop: that is a
 *     wiring or bus problem, not a calibration you can salvage.
 *
 *   - Repeat the whole procedure three times, starting the rotor somewhere
 *     different each time. The offsets should agree within a few hundredths
 *     of a radian. They will not be identical, because friction stops the
 *     rotor slightly short of true alignment from whichever side it
 *     approached. Averaging the runs cancels most of that.
 *
 *   - If two runs disagree by roughly 2*pi/pole_pairs mechanical, the rotor
 *     settled on a different magnet pole. That is fine and expected: every
 *     pole is electrically identical, so both answers work.
 *
 *   - Then command a small q-axis current with the rotor free. It should turn
 *     smoothly and pull hard. If it draws the current you asked for and turns
 *     weakly, or fights itself, the offset is wrong - see the table this
 *     example prints.
 */
#include "foc.h"
#include "upid.h"

#include <math.h>
#include <stdio.h>

#define PI		3.14159265358979323846f

#define POLE_PAIRS	7
#define V_DC		24.0f
#define V_ALIGN		1.5f		/* alignment voltage, volts */
#define R_PHASE		1.0f		/* winding resistance, ohms */
#define L_PHASE		0.001f		/* winding inductance, henries */
#define DT		100e-6f		/* 10 kHz control loop */
#define TARGET_Q	2.0f		/* amps of torque current to test with */

/*
 * The simulated motor. On a bench this is the physical world; here it is one
 * secret number: the mechanical angle the encoder happens to report once the
 * rotor has pulled itself round to electrical zero. Nothing outside these two
 * functions is allowed to look at it, which is the whole point.
 */
#define SECRET_ALIGNED_MECH	0.1234f

static void hold_vector_at_electrical_zero(float v_align, struct foc_abc *duty)
{
	struct foc_dq v_dq = { 0.0f, 0.0f };
	struct foc_ab v_ab;

	v_dq.d = v_align;			/* all flux, no torque */
	v_ab = foc_park_inv(v_dq, 0.0f);	/* at electrical angle zero */
	foc_svm(v_ab, V_DC, duty);
}

static float encoder_read_mech(void)
{
	/* Whatever the rotor did, it ended up here. */
	return SECRET_ALIGNED_MECH;
}

/*
 * One phase of an ideal motor at standstill: L di/dt = v - R i. Enough to
 * show where the current ends up, which is all this example claims.
 * ponytail: no back-EMF and no saliency; add them if you want to study
 * behaviour above standstill, not to reproduce the table below.
 */
static struct foc_dq settle(float offset_used, float target_q)
{
	/* The rotor is still where alignment left it, so its true electrical
	 * angle is zero. Every frame conversion below uses that, and the
	 * controller only agrees when theta_offset is right. */
	struct foc_dq i_true = { 0.0f, 0.0f };
	struct upid_cfg pcfg;
	struct upid pid_d, pid_q;
	struct foc_cfg cfg;
	struct foc_ctrl c;
	int k;

	upid_cfg_init(&pcfg);
	pcfg.kp = L_PHASE * 2000.0f;		/* kp = L w, ki = R w */
	pcfg.ki = R_PHASE * 2000.0f;
	pcfg.o_min = -foc_v_max(V_DC);
	pcfg.o_max = foc_v_max(V_DC);
	upid_create(&pid_d, &pcfg);
	upid_create(&pid_q, &pcfg);

	foc_cfg_init(&cfg);
	cfg.pole_pairs = POLE_PAIRS;
	cfg.v_dc = V_DC;
	cfg.theta_offset = offset_used;
	foc_create(&c, &cfg, &pid_d, &pid_q);

	for (k = 0; k < 1000; k++) {
		struct foc_dq tgt = { 0.0f, 0.0f };
		struct foc_abc duty, i_abc;
		struct foc_ab v_ab;
		struct foc_dq v_true;

		tgt.q = target_q;

		/* The rotor is really at electrical zero, held there by the
		 * clamp, so the true frame is the frame at theta_e = 0. */
		i_abc = foc_clarke_inv(foc_park_inv(i_true, 0.0f));

		foc_spin(&c, tgt, i_abc, SECRET_ALIGNED_MECH, DT, &duty);

		v_ab = foc_clarke(duty);
		v_ab.al *= V_DC;
		v_ab.be *= V_DC;
		v_true = foc_park(v_ab, 0.0f);

		i_true.d += (v_true.d - R_PHASE * i_true.d) * DT / L_PHASE;
		i_true.q += (v_true.q - R_PHASE * i_true.q) * DT / L_PHASE;
	}
	return i_true;
}

int main(void)
{
	static const float err_deg[] = { 0.0f, 15.0f, 45.0f, 90.0f };
	struct foc_abc duty;
	struct foc_cfg cfg;
	float mech, offset;
	unsigned n;

	printf("foc alignment - simulated motor, no hardware\n\n");

	/* Step 3: hold the rotor at electrical zero. */
	hold_vector_at_electrical_zero(V_ALIGN, &duty);
	printf("  holding %.2f V at electrical zero\n", (double)V_ALIGN);
	printf("  duties           %.3f  %.3f  %.3f\n",
	       (double)duty.a, (double)duty.b, (double)duty.c);
	printf("  the rotor swings round to phase A and stops there\n\n");

	/* Step 5: read the encoder and convert with the offset still zero. */
	foc_cfg_init(&cfg);
	cfg.pole_pairs = POLE_PAIRS;
	cfg.v_dc = V_DC;
	cfg.theta_offset = 0.0f;

	mech = encoder_read_mech();
	offset = foc_elec_angle(&cfg, mech);

	printf("  encoder reads    %.4f rad mechanical\n", (double)mech);
	printf("  pole pairs       %d\n", POLE_PAIRS);
	printf("  theta_offset     %.4f rad electrical   <- store this\n\n",
	       (double)offset);

	cfg.theta_offset = offset;
	printf("  check: with it, electrical angle here is %.4f rad (want 0)\n\n",
	       (double)foc_elec_angle(&cfg, mech));

	/* And what being wrong costs. */
	printf("  commanding %.1f A of torque current:\n\n", (double)TARGET_Q);
	printf("    offset error   torque current   current drawn\n");
	for (n = 0; n < sizeof err_deg / sizeof err_deg[0]; n++) {
		float err = err_deg[n] * PI / 180.0f;
		struct foc_dq i = settle(foc_wrap(offset + err), TARGET_Q);
		float mag = sqrtf(i.d * i.d + i.q * i.q);

		printf("    %5.0f deg        %6.3f A         %6.3f A\n",
		       (double)err_deg[n], (double)i.q, (double)mag);
	}

	printf("\n  The current drawn never changes - only the torque does.\n");
	printf("  Torque falls off as cos(error), so 90 degrees out the motor\n");
	printf("  pulls its full current, gets hot, and does not turn. The\n");
	printf("  current loop reports perfect tracking throughout, because in\n");
	printf("  the frame it believes in, the tracking IS perfect.\n");
	return 0;
}
