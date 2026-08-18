#include "imp.h"
#include <math.h>
#include <string.h>

/* Soft deadband: subtract it rather than snapping to zero, so the output
 * stays continuous as the force crosses the threshold. */
static double deadband(double f, double db)
{
    if (db <= 0.0) return f;
    if (f >  db) return f - db;
    if (f < -db) return f + db;
    return 0.0;
}

/* Symmetric clamp. A limit of 0 (or less) means "no limit". */
static double clamp(double v, double lim)
{
    if (lim <= 0.0) return v;
    if (v >  lim) return  lim;
    if (v < -lim) return -lim;
    return v;
}

static int gains_ok(const imp *s)
{
    int i;
    if (s->n < 1 || s->n > IMP_MAX_DOF) return 0;
    for (i = 0; i < s->n; i++) {
        const imp_axis *a = &s->axis[i];
        if (a->m < 0.0 || a->d < 0.0 || a->k < 0.0) return 0;
        if (a->m == 0.0 && a->d <= 0.0) return 0;   /* no mass and no damping */
    }
    return 1;
}

int imp_admittance(imp *s, const double *f_ext, double dt)
{
    int i;

    if (!s || !f_ext || !(dt > 0.0)) return IMP_EARG;
    if (!gains_ok(s)) return IMP_EARG;

    for (i = 0; i < s->n; i++) {
        const imp_axis *a = &s->axis[i];
        double f = clamp(deadband(f_ext[i], a->f_deadband), a->f_max);
        double x = s->offset[i];
        double v = s->rate[i];

        if (a->m > 0.0) {
            double acc = (f - a->d * v - a->k * x) / a->m;
            v += acc * dt;                  /* semi-implicit Euler: v first, */
        } else {
            v = (f - a->k * x) / a->d;      /* massless: velocity is direct */
        }
        v = clamp(v, a->v_max);
        x += v * dt;                        /* then x with the fresh v */

        if (a->x_max > 0.0 && (x > a->x_max || x < -a->x_max)) {
            x = clamp(x, a->x_max);
            v = 0.0;                        /* stop pushing into the stop */
        }

        s->offset[i] = x;
        s->rate[i] = v;
    }
    return IMP_OK;
}

int imp_impedance(const imp *s, const double *x, const double *v,
                  const double *xd, const double *vd, double *f_out)
{
    int i;

    if (!s || !x || !xd || !f_out) return IMP_EARG;
    if (!gains_ok(s)) return IMP_EARG;

    for (i = 0; i < s->n; i++) {
        const imp_axis *a = &s->axis[i];
        double dx = xd[i] - x[i];
        double dv = (vd ? vd[i] : 0.0) - (v ? v[i] : 0.0);
        f_out[i] = clamp(a->k * dx + a->d * dv, a->f_max);
    }
    return IMP_OK;
}

void imp_reset(imp *s)
{
    if (!s) return;
    memset(s->offset, 0, sizeof s->offset);
    memset(s->rate, 0, sizeof s->rate);
}

double imp_damping(double m, double k, double zeta)
{
    if (m <= 0.0 || k <= 0.0 || zeta <= 0.0) return 0.0;
    return 2.0 * zeta * sqrt(k * m);
}
