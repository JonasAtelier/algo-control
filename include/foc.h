/*
 * foc - field-oriented control maths for a three-phase motor.
 *
 * No platform headers, no hardware access, no allocation, no globals. Phase
 * currents and a rotor angle go in, three duty cycles come out. What produces
 * the currents and what does something with the duties is not this library's
 * business.
 *
 * Two layers, and you can take only the first:
 *
 *   the transforms   Clarke, Park, their inverses, and space-vector
 *                    modulation. Pure functions. src/foc.c compiles on its
 *                    own with nothing but libm.
 *
 *   the current loop foc_spin(), which wires those together around two PID
 *                    controllers from upid. Lives in src/foc_loop.c, the only
 *                    file that includes upid.h. Skip that file and the
 *                    dependency disappears with it.
 *
 * ANGLES ARE FLOAT RADIANS, ELECTRICAL, wrapped to [-pi, pi). Every angle
 * this library takes or returns follows that rule, with one exception that
 * says so in its own comment: foc_elec_angle() and foc_spin() take a
 * MECHANICAL angle, because that is what an encoder reports, and convert it.
 * There is no fixed-point angle type and no degrees anywhere.
 *
 * C99, libm only.
 */
#ifndef FOC_FOC_H
#define FOC_FOC_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Result of an operation that can fail. The transforms cannot fail and so
 * return their answer directly. */
typedef enum {
	FOC_OK = 0,
	FOC_EINVAL,	/* NULL argument, a value that was not finite, or a
			   configuration that cannot be honoured */
	FOC_ESAT,	/* the request exceeded the bus and was scaled down to
			   fit. Duties were still written and are usable. */
} foc_sta;

/* Three phase quantities. Currents, voltages or duty cycles - the transforms
 * do not care which, only that all three are in the same unit. */
struct foc_abc {
	float a;
	float b;
	float c;
};

/* The stationary two-axis frame. alpha lies along phase A, beta leads it by
 * 90 electrical degrees. */
struct foc_ab {
	float al;
	float be;
};

/* The rotor frame. d points along the rotor magnets, so d current produces
 * flux and no torque; q is 90 electrical degrees ahead of it, so q current
 * produces torque and no flux. Driving d to zero and q to what you want is
 * the whole idea. */
struct foc_dq {
	float d;
	float q;
};

/*
 * ---------------------------------------------------------------------------
 * The transforms. Pure, total, dependency-free.
 * ---------------------------------------------------------------------------
 */

/*
 * Three phases to the stationary frame, AMPLITUDE-INVARIANT:
 *
 *     alpha = (2a - b - c) / 3        beta = (b - c) / sqrt(3)
 *
 * The 2/3 matters. A balanced set of peak amplitude I maps to a vector of
 * magnitude I, which is why a phase current of 10 A peak reads as 10 on the
 * d-q axes rather than 12.25. The other common convention, power-invariant,
 * scales by sqrt(2/3) instead and every number it produces differs from these
 * by sqrt(3/2). If your textbook disagrees with this library by a factor of
 * about 1.22, that is which one, and neither is wrong.
 *
 * The zero-sequence component (a + b + c) is discarded, because a three-wire
 * motor cannot carry one. foc_clarke_inv() therefore always returns a set
 * summing to zero, and round-tripping a set that does not sum to zero will
 * not give you back what you put in. That is the transform being correct, not
 * a loss of precision.
 */
struct foc_ab foc_clarke(struct foc_abc x);

/* Stationary frame back to three phases. The result sums to zero. */
struct foc_abc foc_clarke_inv(struct foc_ab x);

/*
 * Stationary frame to the rotor frame, given the rotor's electrical angle:
 *
 *     d =  alpha cos(theta) + beta sin(theta)
 *     q = -alpha sin(theta) + beta cos(theta)
 *
 * A vector rotating at the same rate as the rotor becomes two constants.
 * theta_e need not be wrapped; sinf and cosf handle any finite angle.
 */
struct foc_dq foc_park(struct foc_ab x, float theta_e);

/* Rotor frame back to the stationary frame. */
struct foc_ab foc_park_inv(struct foc_dq x, float theta_e);

/* Wrap any finite angle in radians to [-pi, pi). Cheap, and correct for
 * inputs many revolutions out of range. */
float foc_wrap(float rad);

/*
 * The largest voltage vector magnitude a bus of v_dc can produce without
 * distortion: v_dc / sqrt(3), about 0.577 v_dc.
 *
 * Use it for the output limits of your current-loop PIDs. Letting them ask
 * for more than this just makes foc_svm() scale the answer back down, which
 * is integral windup wearing a disguise.
 */
float foc_v_max(float v_dc);

/*
 * A voltage vector to three duty cycles, by min-max midpoint injection.
 *
 * The requested vector is expanded to three phase voltages, then the midpoint
 * of the largest and smallest is subtracted from all three before scaling to
 * duties. That common-mode shift cancels in every phase-to-phase voltage, so
 * the motor sees exactly the vector you asked for, while the duties sit
 * centred in [0, 1] and reach 15.5% further before clipping. The output is
 * the same as classical sector-based SVPWM with symmetric dwell times, and
 * has no sector boundaries to get wrong.
 *
 * duty is written on FOC_OK and FOC_ESAT, and is always within [0, 1].
 *
 * Returns FOC_ESAT when the request was longer than foc_v_max(v_dc), in which
 * case the vector was scaled to that length with its angle preserved. Treat
 * it as "the bus ran out", not as an error: the duties are still the best
 * available answer.
 */
foc_sta foc_svm(struct foc_ab v, float v_dc, struct foc_abc *duty);

/*
 * ---------------------------------------------------------------------------
 * The current loop. Needs upid; lives in src/foc_loop.c.
 * ---------------------------------------------------------------------------
 */

/* Declared, never defined here. Only src/foc_loop.c includes upid.h, so the
 * transforms above compile with no trace of upid on the include path. */
struct upid;

/*
 * What this library cannot work out for itself.
 *
 * theta_offset is the one that will cost you an afternoon. See the note on
 * foc_elec_angle() before you guess at it.
 */
struct foc_cfg {
	int pole_pairs;		/* magnet pole pairs; electrical revolutions
				   per mechanical revolution. 7 and 11 are
				   common on hobby gimbal and drone motors. */
	float theta_offset;	/* electrical radians, see foc_elec_angle() */
	bool reverse;		/* true when the encoder counts the opposite
				   way round from the phase order A-B-C */
	float v_dc;		/* bus voltage, volts, > 0 */
};

/*
 * Controller state. Declare one wherever you like - on the stack, in a
 * static, inside your own struct - and hand its address to every call.
 * Nothing here is allocated.
 *
 * The two PIDs are yours: you declare them, you upid_create() them, you tune
 * them, and foc only calls upid_spin() on them. That is deliberate. Wrapping
 * them would mean re-exporting every knob upid already has.
 *
 * The fields are yours to read and nothing else. Until foc_create() has run
 * the contents mean nothing, and every call refuses to touch it.
 */
struct foc_ctrl {
	uint32_t marker;	/* set by foc_create; proves it ran */
	struct foc_cfg cfg;
	struct upid *pid_d;
	struct upid *pid_q;
	float theta_e;		/* electrical angle used by the last foc_spin */
	struct foc_dq i_dq;	/* measured current, last foc_spin */
	struct foc_dq v_dq;	/* voltage the PIDs asked for, last foc_spin */
};

/* Load defaults into cfg: 7 pole pairs, no offset, forward, a 12 V bus.
 * Start here rather than from a zeroed struct - zero pole pairs and a zero
 * bus are both refused, so all-zero is not a valid cfg. */
foc_sta foc_cfg_init(struct foc_cfg *cfg);

/*
 * Initialise, or reinitialise, a controller. Validates cfg, copies it in, and
 * adopts the two PIDs, which must already be upid_create()d and must outlive
 * the controller.
 *
 * Returns FOC_EINVAL for a cfg that cannot be honoured - pole_pairs below 1,
 * v_dc not positive, a non-finite offset - or a NULL anywhere, in which case
 * the controller stays unrunnable rather than running on bad numbers.
 */
foc_sta foc_create(struct foc_ctrl *c, const struct foc_cfg *cfg,
		   struct upid *pid_d, struct upid *pid_q);

/*
 * Mechanical shaft angle to rotor electrical angle:
 *
 *     theta_e = wrap(pole_pairs * (reverse ? -theta_m : theta_m) - theta_offset)
 *
 * THE OFFSET CANNOT BE COMPUTED. It is the electrical angle your encoder
 * happens to read when the rotor's d-axis is aligned with phase A, and it
 * depends on how the magnet was glued to the shaft. You measure it once, on
 * the assembled motor, by forcing the rotor to a known electrical angle and
 * reading the encoder back. examples/align.c does exactly that.
 *
 * Get it wrong and the motor still draws every amp you ask for; it just spends
 * them on the wrong axis. At 90 degrees out, all of that current makes flux
 * and none of it makes torque, so the motor sits there getting hot while the
 * current loop reports perfect tracking - because in the frame it believes
 * in, the tracking is perfect. It is the failure that looks like dead
 * hardware, and the current loop cannot detect it.
 *
 * theta_m is MECHANICAL radians, the raw shaft angle, and need not be wrapped.
 */
float foc_elec_angle(const struct foc_cfg *cfg, float theta_mech);

/*
 * One control sample: measure, rotate, regulate, rotate back, modulate.
 *
 *   i_abc + theta_mech  ->  Clarke  ->  Park  ->  two PIDs  ->  inverse Park
 *                       ->  space-vector modulation  ->  duty
 *
 * target.d is normally zero, which is the "use every amp for torque" case.
 * target.q is your torque command. i_abc is the measured phase currents in
 * amps; if you only sample two, pass c = -(a + b). dt_s is the elapsed time
 * in seconds, exactly as upid_spin() wants it.
 *
 * Returns FOC_ESAT when the bus ran out, as foc_svm() does, with duty still
 * written. Returns FOC_EINVAL for a NULL, a non-finite input, a controller
 * foc_create() has not run on, or a dt the PIDs rejected.
 */
foc_sta foc_spin(struct foc_ctrl *c, struct foc_dq target, struct foc_abc i_abc,
		 float theta_mech, float dt_s, struct foc_abc *duty);

#ifdef __cplusplus
}
#endif

#endif /* FOC_FOC_H */
