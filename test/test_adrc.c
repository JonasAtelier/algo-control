/* Self-check: assert-based, no framework. Build with `make check`.
 *
 * Every plant here is stepped with the same forward Euler the controller
 * uses, so a test failure means the control law changed, not the sim. */
#include "adrc.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_bad_cfg(void)
{
    adrc s;
    adrc_cfg good = { 1, 2.0, 20.0, 100.0, 0.0, 0.0 };
    adrc_cfg bad;

    assert(adrc_init(&s, &good) == ADRC_OK);
    assert(adrc_init(NULL, &good) == ADRC_EARG);
    assert(adrc_init(&s, NULL) == ADRC_EARG);

    bad = good; bad.order = 3;  assert(adrc_init(&s, &bad) == ADRC_EARG);
    bad = good; bad.b0 = 0.0;   assert(adrc_init(&s, &bad) == ADRC_EARG);
    bad = good; bad.wc = 0.0;   assert(adrc_init(&s, &bad) == ADRC_EARG);
    bad = good; bad.wo = -1.0;  assert(adrc_init(&s, &bad) == ADRC_EARG);
    bad = good; bad.u_min = 1.0; bad.u_max = -1.0;
                                assert(adrc_init(&s, &bad) == ADRC_EARG);

    bad = good; bad.wo = 0.0;   assert(adrc_init(&s, &bad) == ADRC_OK);
    assert(fabs(s.cfg.wo - 5.0 * good.wc) < 1e-12);   /* 0 selects 5 x wc */
}

/* Order-1 plant  y' = b0*u + d  with a constant load the controller was
 * never told about. It should still land on the setpoint, and z[1] should
 * end up holding the load. */
static void test_order1_tracks_and_estimates(void)
{
    adrc_cfg cfg = { .order = 1, .b0 = 2.0, .wc = 20.0, .wo = 100.0 };
    const double d = -3.0, r = 1.0, dt = 1e-3;
    adrc s;
    double y = 0.0;
    int k;

    assert(adrc_init(&s, &cfg) == ADRC_OK);
    for (k = 0; k < 1000; k++) {                  /* 1 s */
        double u = adrc_update(&s, r, y, dt);
        y += dt * (cfg.b0 * u + d);
    }
    assert(fabs(y - r) < 1e-2);
    assert(fabs(s.z[1] - d) < 0.1);
}

/* Run the order-1 plant for 1 s against a limited output, checking the
 * limits hold on every single step, and hand back where it ended up. */
static double run_clamped(double u_min, double u_max, double r, double d)
{
    adrc_cfg cfg = { .order = 1, .b0 = 2.0, .wc = 20.0, .wo = 100.0 };
    const double dt = 1e-3;
    adrc s;
    double y = 0.0;
    int k;

    cfg.u_min = u_min;
    cfg.u_max = u_max;
    assert(adrc_init(&s, &cfg) == ADRC_OK);
    for (k = 0; k < 1000; k++) {
        double u = adrc_update(&s, r, y, dt);
        assert(u >= u_min && u <= u_max);
        y += dt * (cfg.b0 * u + d);
        /* Both runs start hard against a limit, so this is where windup
         * would show: feed the observer the output it asked for instead of
         * the one it got and this run overshoots by 22%. r is never 0, and
         * dividing by it makes the check read the same for a negative
         * setpoint as a positive one. */
        assert((y - r) / r < 0.02);
    }
    return y;
}

/* One run limited above, one below. Note that u_min stays 0 in the first and
 * u_max stays 0 in the second - "both 0 for none" means only both, so each
 * run is limited on one side and pinned at zero on the other. Both start
 * hard against their limit and both loads leave room, so both settle. */
static void test_order1_clamps(void)
{
    assert(fabs(run_clamped( 0.0, 5.0,  1.0, -3.0) - 1.0) < 1e-2);
    assert(fabs(run_clamped(-5.0, 0.0, -1.0,  3.0) + 1.0) < 1e-2);
}

/* Order-2 plant  y'' = b0*u + d. */
static void test_order2_tracks_and_estimates(void)
{
    adrc_cfg cfg = { .order = 2, .b0 = 1.0, .wc = 10.0, .wo = 60.0 };
    const double d = 2.5, r = 1.0, dt = 1e-3;
    adrc s;
    double y = 0.0, v = 0.0;
    int k;

    assert(adrc_init(&s, &cfg) == ADRC_OK);
    for (k = 0; k < 3000; k++) {                  /* 3 s */
        double u = adrc_update(&s, r, y, dt);
        v += dt * (cfg.b0 * u + d);
        y += dt * v;
    }
    assert(fabs(y - r) < 1e-2);
    assert(fabs(s.z[2] - d) < 0.1);
}

static void test_reset(void)
{
    adrc_cfg cfg = { .order = 1, .b0 = 2.0, .wc = 20.0, .wo = 100.0 };
    adrc s;
    double u;

    assert(adrc_init(&s, &cfg) == ADRC_OK);
    adrc_update(&s, 1.0, 0.0, 1e-3);              /* dirty the state */

    adrc_reset(&s, 7.5);
    assert(s.z[0] == 7.5);
    assert(s.z[1] == 0.0 && s.z[2] == 0.0 && s.u == 0.0);

    u = adrc_update(&s, 7.5, 7.5, 1e-3);
    assert(isfinite(u));
}

/* A bad sample must change nothing at all. */
static void test_bad_sample_holds_output(void)
{
    adrc_cfg cfg = { .order = 1, .b0 = 2.0, .wc = 20.0, .wo = 100.0 };
    adrc s;
    double u, z0, z1;

    assert(adrc_init(&s, &cfg) == ADRC_OK);
    u  = adrc_update(&s, 1.0, 0.0, 1e-3);
    z0 = s.z[0];
    z1 = s.z[1];

    assert(adrc_update(&s, 1.0, NAN, 1e-3) == u);
    assert(adrc_update(&s, NAN, 0.0, 1e-3) == u);
    assert(adrc_update(&s, 1.0, 0.0, 0.0)  == u);   /* dt must be > 0 */
    assert(s.z[0] == z0 && s.z[1] == z1);
}

/* A load lands on a loop that had settled. Two things must hold: the output
 * comes back, and the estimate walks to the disturbance without shooting
 * past it.
 *
 * The recovery window is loose on purpose. Measured settling to 1e-3 after
 * the step is 0.224 s with the correct observer gains and 0.177 s with l1
 * mistuned from 2*wo to wo, so no window separates the two - the mistuned
 * observer is underdamped and therefore *faster* here. What separates them
 * is overshoot: the correct, critically damped observer walks the estimate
 * to -3 and stops, 0.00% past it, while the mistuned one rings 19.63% past.
 * That is the assertion pinning the gain pair. */
static void test_load_step_recovery(void)
{
    adrc_cfg cfg = { .order = 1, .b0 = 2.0, .wc = 20.0, .wo = 100.0 };
    const double d = -3.0, r = 1.0, dt = 1e-3;
    adrc s;
    double y = 0.0, worst_overshoot = 0.0;
    int k;

    assert(adrc_init(&s, &cfg) == ADRC_OK);
    for (k = 0; k < 2000; k++) {              /* 1 s to settle, then the load */
        double t    = k * dt;
        double load = (t >= 1.0) ? d : 0.0;
        double u    = adrc_update(&s, r, y, dt);

        y += dt * (cfg.b0 * u + load);

        if (t >= 1.4) assert(fabs(y - r) < 1e-3);      /* 0.4 s to recover */
        if (t >= 1.0 && d - s.z[1] > worst_overshoot)
            worst_overshoot = d - s.z[1];
    }
    assert(worst_overshoot < 0.05);           /* 1.7% of d; the mutant hits 0.59 */
    assert(fabs(s.z[1] - d) < 0.1);
}

int main(void)
{
    test_bad_cfg();
    test_order1_tracks_and_estimates();
    test_order1_clamps();
    test_order2_tracks_and_estimates();
    test_reset();
    test_bad_sample_holds_output();
    test_load_step_recovery();
    printf("all tests passed\n");
    return 0;
}
