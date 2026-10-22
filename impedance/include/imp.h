/* imp.h - portable impedance and admittance control.
 *
 * Makes a robot behave like a virtual mass-spring-damper against its
 * environment, so it yields when pushed instead of fighting back.
 *
 * Axes are independent and unit-agnostic: 6 Cartesian axes, 7 joints, or a
 * single linear stage - the library never asks what an axis means, only that
 * you stay consistent (N with m, Nm with rad).
 *
 * C99, no dependencies beyond libm, no malloc, no globals, no I/O.
 */
#ifndef IMP_H
#define IMP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Max axes. Override at build time: -DIMP_MAX_DOF=3 */
#ifndef IMP_MAX_DOF
#define IMP_MAX_DOF 6
#endif

#define IMP_OK    0
#define IMP_EARG (-1)   /* bad argument, or physically impossible gains */

/* Per-axis behaviour. Zero any limit to disable it. */
typedef struct {
    double m;           /* virtual mass. 0 = pure damper (admittance only) */
    double d;           /* virtual damping. Must be > 0 when m is 0 */
    double k;           /* virtual stiffness. 0 = free floating */

    double f_deadband;  /* ignore force below this - real sensors are noisy */
    double f_max;       /* clamp the force this axis will react to */
    double x_max;       /* clamp how far the axis may deflect */
    double v_max;       /* clamp how fast it may deflect */
} imp_axis;

/* One controller. Set it up with a plain initialiser:
 *
 *   imp s = { .n = 1, .axis = {{ .m = 2, .d = 30, .k = 100 }} };
 *
 * offset[] and rate[] are the live state - read them, do not write them.
 * Use imp_reset() to clear them. */
typedef struct {
    int n;                          /* number of axes in use, 1..IMP_MAX_DOF */
    imp_axis axis[IMP_MAX_DOF];

    double offset[IMP_MAX_DOF];     /* deflection away from the desired pose */
    double rate[IMP_MAX_DOF];       /* how fast that deflection is changing */
} imp;

/* Admittance: force in, motion out. For a position-controlled robot.
 *
 * Feed the measured external force and the time since the last call. Then
 * command  desired_position[i] + s.offset[i]  to the robot.
 *
 * dt must be > 0, and small next to the virtual system: keep it well under
 * sqrt(m/k). Returns IMP_OK or IMP_EARG. */
int imp_admittance(imp *s, const double *f_ext, double dt);

/* Impedance: motion in, force out. For a torque-controlled robot.
 *
 *   f = k * (xd - x) + d * (vd - v),  clamped to f_max
 *
 * v and vd may be NULL, meaning zero. Uses d and k only - the virtual mass
 * m belongs to imp_admittance. Returns IMP_OK or IMP_EARG. */
int imp_impedance(const imp *s, const double *x, const double *v,
                  const double *xd, const double *vd, double *f_out);

/* Clear the deflection state. Call before re-enabling the controller, so it
 * does not resume with a stale offset and jolt the robot. */
void imp_reset(imp *s);

/* Damping for a chosen behaviour, given mass and stiffness:
 * zeta = 1 is critically damped (no overshoot), < 1 springs back and
 * wobbles, > 1 is sluggish. Returns 2 * zeta * sqrt(k * m). */
double imp_damping(double m, double k, double zeta);

#ifdef __cplusplus
}
#endif
#endif /* IMP_H */
