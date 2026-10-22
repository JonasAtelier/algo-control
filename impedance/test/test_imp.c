/* Self-check: assert-based, no framework. Build with `make check`. */
#include "imp.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.001

static imp spring(double m, double k, double zeta)
{
    imp s = { 0 };
    s.n = 1;
    s.axis[0].m = m;
    s.axis[0].k = k;
    s.axis[0].d = imp_damping(m, k, zeta);
    return s;
}

/* Run a constant force for `seconds`, return the largest offset seen. */
static double drive(imp *s, double f, double seconds)
{
    double peak = 0.0;
    int i, n = (int)(seconds / DT);
    for (i = 0; i < n; i++) {
        assert(imp_admittance(s, &f, DT) == IMP_OK);
        if (fabs(s->offset[0]) > fabs(peak)) peak = s->offset[0];
    }
    return peak;
}

/* A steady push must settle at exactly force / stiffness. */
static void test_steady_state(void)
{
    imp s = spring(2.0, 100.0, 1.0);
    drive(&s, 10.0, 5.0);
    assert(fabs(s.offset[0] - 0.1) < 1e-4);      /* 10 N / 100 N/m */
    assert(fabs(s.rate[0]) < 1e-4);              /* and stopped moving */
}

static void test_damping_shapes_response(void)
{
    imp crit = spring(2.0, 100.0, 1.0);
    imp soft = spring(2.0, 100.0, 0.2);

    assert(drive(&crit, 10.0, 3.0) <= 0.1 * 1.001);   /* no overshoot */
    assert(drive(&soft, 10.0, 3.0) >  0.1 * 1.20);    /* clear overshoot */

    assert(fabs(imp_damping(2.0, 100.0, 1.0) - 2.0 * sqrt(200.0)) < 1e-9);
    assert(imp_damping(0.0, 100.0, 1.0) == 0.0);      /* nonsense -> 0 */
}

/* m = 0 is a pure damper: constant force gives constant speed. */
static void test_massless_damper(void)
{
    imp s = { 0 };
    s.n = 1;
    s.axis[0].d = 50.0;
    drive(&s, 10.0, 1.0);
    assert(fabs(s.rate[0] - 0.2) < 1e-9);        /* 10 N / 50 Ns/m */
    assert(fabs(s.offset[0] - 0.2) < 1e-3);      /* 0.2 m/s for 1 s */
}

static void test_deadband(void)
{
    imp s = spring(2.0, 100.0, 1.0);
    s.axis[0].f_deadband = 2.0;

    drive(&s, 1.5, 1.0);
    assert(s.offset[0] == 0.0);                  /* below the band: ignored */

    drive(&s, 12.0, 5.0);
    assert(fabs(s.offset[0] - 0.1) < 1e-4);      /* 12 N - 2 N band = 10 N */
}

static void test_limits(void)
{
    imp s = spring(1.0, 0.0, 1.0);               /* free floating */
    s.axis[0].d = 10.0;
    s.axis[0].x_max = 0.05;
    s.axis[0].v_max = 0.30;

    drive(&s, 500.0, 2.0);
    assert(fabs(s.offset[0]) <= 0.05 + 1e-12);   /* travel clamped */
    assert(s.rate[0] == 0.0);                    /* and it stopped pushing */

    s = spring(1.0, 0.0, 1.0);
    s.axis[0].d = 10.0;
    s.axis[0].v_max = 0.30;
    drive(&s, 500.0, 0.5);
    assert(fabs(s.rate[0]) <= 0.30 + 1e-12);     /* speed clamped */

    s = spring(2.0, 100.0, 1.0);
    s.axis[0].f_max = 5.0;
    drive(&s, 50.0, 5.0);
    assert(fabs(s.offset[0] - 0.05) < 1e-4);     /* force clamped to 5 N */
}

static void test_impedance_law(void)
{
    imp s = spring(2.0, 100.0, 1.0);
    double x = 0.1, xd = 0.3, v = 0.0, f;

    s.axis[0].d = 10.0;
    assert(imp_impedance(&s, &x, NULL, &xd, NULL, &f) == IMP_OK);
    assert(fabs(f - 20.0) < 1e-12);              /* 100 * 0.2 */

    v = 0.5;
    assert(imp_impedance(&s, &x, &v, &xd, NULL, &f) == IMP_OK);
    assert(fabs(f - 15.0) < 1e-12);              /* 20 - 10 * 0.5 */

    s.axis[0].f_max = 12.0;
    imp_impedance(&s, &x, &v, &xd, NULL, &f);
    assert(fabs(f - 12.0) < 1e-12);              /* clamped */
}

static void test_reset(void)
{
    imp s = spring(2.0, 100.0, 1.0);
    drive(&s, 10.0, 0.2);
    assert(s.offset[0] != 0.0);
    imp_reset(&s);
    assert(s.offset[0] == 0.0 && s.rate[0] == 0.0);
    imp_reset(NULL);                             /* must not crash */
}

static void test_multi_axis(void)
{
    imp s = { 0 };
    double f[3] = { 10.0, -10.0, 0.0 };
    int i;

    s.n = 3;
    for (i = 0; i < 3; i++) {
        s.axis[i].m = 1.0;
        s.axis[i].k = 100.0;
        s.axis[i].d = imp_damping(1.0, 100.0, 1.0);
    }
    for (i = 0; i < 5000; i++) assert(imp_admittance(&s, f, DT) == IMP_OK);

    assert(fabs(s.offset[0] - 0.1) < 1e-4);
    assert(fabs(s.offset[1] + 0.1) < 1e-4);      /* axes are independent */
    assert(s.offset[2] == 0.0);
}

static void test_bad_args(void)
{
    imp s = spring(2.0, 100.0, 1.0);
    double f = 1.0, x = 0.0;

    assert(imp_admittance(NULL, &f, DT) == IMP_EARG);
    assert(imp_admittance(&s, NULL, DT) == IMP_EARG);
    assert(imp_admittance(&s, &f, 0.0) == IMP_EARG);
    assert(imp_admittance(&s, &f, -DT) == IMP_EARG);

    s.n = 0;              assert(imp_admittance(&s, &f, DT) == IMP_EARG);
    s.n = IMP_MAX_DOF + 1; assert(imp_admittance(&s, &f, DT) == IMP_EARG);

    s = spring(2.0, 100.0, 1.0);
    s.axis[0].k = -1.0;   assert(imp_admittance(&s, &f, DT) == IMP_EARG);

    s = spring(0.0, 0.0, 1.0);
    s.axis[0].d = 0.0;    assert(imp_admittance(&s, &f, DT) == IMP_EARG);

    s = spring(2.0, 100.0, 1.0);
    assert(imp_impedance(&s, NULL, NULL, &x, NULL, &f) == IMP_EARG);
    assert(imp_impedance(&s, &x, NULL, &x, NULL, NULL) == IMP_EARG);
}

int main(void)
{
    test_steady_state();
    test_damping_shapes_response();
    test_massless_damper();
    test_deadband();
    test_limits();
    test_impedance_law();
    test_reset();
    test_multi_axis();
    test_bad_args();
    printf("all tests passed\n");
    return 0;
}
