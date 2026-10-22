/* adrc.h - linear active disturbance rejection control.
 *
 * A PID alternative that estimates whatever is pushing the plant around
 * (load, friction, model error) and cancels it, so one gain pair works
 * across operating points. Gao's bandwidth parameterisation: two knobs.
 *
 * C99, no dependencies beyond libm, no malloc, no globals, no I/O.
 */
#ifndef ADRC_H
#define ADRC_H

#ifdef __cplusplus
extern "C" {
#endif

/* Return codes. */
#define ADRC_OK    0
#define ADRC_EARG (-1)   /* bad argument, or a setting that cannot be honoured */

/* What the controller needs to know about your plant, and how hard to push.
 *
 *   adrc_cfg cfg = { .order = 1, .b0 = 250.0, .wc = 30.0 };
 */
typedef struct {
    int    order;    /* 1 or 2: integrators between input and output. Speed loop: 1. Position loop: 2 */
    double b0;       /* plant gain estimate: y^(order) ~= b0 * u + disturbance. Wrong by 2x is fine; wrong sign is not */
    double wc;       /* controller bandwidth, rad/s. Raise for a faster response */
    double wo;       /* observer bandwidth, rad/s. 3 to 10 x wc; 0 selects 5 x wc */
    double u_min;    /* output limits; leave both 0 for none */
    double u_max;
} adrc_cfg;

/* One controller. Declare it wherever you like - nothing is allocated.
 *
 * Every field is yours to read and nothing else. cfg is the copy the
 * controller runs on, so cfg.wo shows the observer bandwidth actually in
 * use after a 0 has been resolved to 5 * wc. */
typedef struct {
    adrc_cfg cfg;
    double   z[3];   /* observer state: [0] output estimate, [1] its rate (order 2 only), [order] total disturbance */
    double   u;      /* last output */
} adrc;

/* Validate the settings and start a controller from them.
 *
 * Requires order 1 or 2, b0 != 0, wc > 0, wo >= 0 and u_min <= u_max;
 * anything else returns ADRC_EARG and leaves s untouched. A wo of 0 is
 * resolved to 5 * wc and stored. The observer starts at zero, so call
 * adrc_reset() before the first sample if the plant is not at zero.
 *
 * Returns ADRC_OK or ADRC_EARG. Until it has returned ADRC_OK, every
 * adrc_update() on this struct is a no-op that returns 0. */
int adrc_init(adrc *s, const adrc_cfg *cfg);

/* One control step. Feed it where you want to be, where you are, and how
 * long since the last call. Returns the output to send to the plant,
 * clamped to the configured limits.
 *
 * dt is in seconds and must be > 0, and small next to the observer:
 * keep wo * dt well under 1 (under 0.2 is comfortable).
 *
 * A setpoint, measurement or dt that is not finite, or a dt <= 0, is
 * refused: the previous output is returned and the observer is left
 * alone, so one bad sensor reading cannot poison the controller. */
double adrc_update(adrc *s, double setpoint, double measurement, double dt);

/* Restart the observer at the current output, with zero disturbance and
 * zero output. Call this before re-enabling a controller that has been
 * off, so it does not resume from a stale estimate and jolt the plant. */
void adrc_reset(adrc *s, double measurement);

#ifdef __cplusplus
}
#endif
#endif /* ADRC_H */
