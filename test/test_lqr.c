/* Self-check: assert-based, no framework. Build with `make check`. */
#include "lqr.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define DT 0.01

/* 1-D double integrator (position, velocity), forward Euler at DT. */
static const double di_A[4] = { 1.0, DT, 0.0, 1.0 };
static const double di_B[2] = { 0.0, DT };
static const double di_Q[4] = { 1.0, 0.0, 0.0, 0.1 };
static const double di_R[1] = { 1.0 };

/* Local matrix helpers, so the residual check below does not lean on the
 * library's own expression for the Riccati update. */

/* C(ra x cb) = A(ra x ca) * B(ca x cb) */
static void mul(int ra, int ca, int cb, const double *A, const double *B, double *C)
{
    int i, j, k;
    for (i = 0; i < ra; i++)
        for (j = 0; j < cb; j++) {
            double s = 0.0;
            for (k = 0; k < ca; k++) s += A[i * ca + k] * B[k * cb + j];
            C[i * cb + j] = s;
        }
}

/* C(ca x cb) = A(ra x ca)' * B(ra x cb) */
static void mul_at(int ra, int ca, int cb, const double *A, const double *B, double *C)
{
    int i, j, k;
    for (i = 0; i < ca; i++)
        for (j = 0; j < cb; j++) {
            double s = 0.0;
            for (k = 0; k < ra; k++) s += A[k * ca + i] * B[k * cb + j];
            C[i * cb + j] = s;
        }
}

/* A = B = Q = R = 1 has a closed form: P is the golden ratio, K = P/(1+P). */
static void test_scalar_closed_form(void)
{
    double A = 1.0, B = 1.0, Q = 1.0, R = 1.0, K = 0.0, P = 0.0;
    double phi = (1.0 + sqrt(5.0)) / 2.0;

    assert(lqr_solve(1, 1, &A, &B, &Q, &R, &K, &P, NULL) == LQR_OK);
    assert(fabs(P - phi) < 1e-9);
    assert(fabs(K - phi / (1.0 + phi)) < 1e-9);
}

static void test_double_integrator_settles(void)
{
    double K[2], x[2] = { 1.0, 0.0 };
    int i, settled = 0;

    assert(lqr_gain(2, 1, di_A, di_B, di_Q, di_R, K, NULL) == LQR_OK);
    assert(isfinite(K[0]) && isfinite(K[1]));

    for (i = 0; i < 2000; i++) {
        lqr_step(2, 1, di_A, di_B, K, x, x);          /* aliasing is allowed */
        if (sqrt(x[0] * x[0] + x[1] * x[1]) < 1e-3) { settled = 1; break; }
    }
    assert(settled);
}

/* The returned P must satisfy the DARE. Checked in its closed-loop form,
 * Q + Acl' P Acl + K' R K - P = 0 with Acl = A - B K, which exercises K too. */
static void test_dare_residual(void)
{
    double K[2], P[4], BK[4], Acl[4], PAcl[4], res[4], RK[2], KRK[4];
    int i;

    assert(lqr_solve(2, 1, di_A, di_B, di_Q, di_R, K, P, NULL) == LQR_OK);

    mul(2, 1, 2, di_B, K, BK);
    for (i = 0; i < 4; i++) Acl[i] = di_A[i] - BK[i];

    mul(2, 2, 2, P, Acl, PAcl);
    mul_at(2, 2, 2, Acl, PAcl, res);        /* res = Acl' P Acl */

    mul(1, 1, 2, di_R, K, RK);
    mul_at(1, 2, 2, K, RK, KRK);            /* KRK = K' R K */

    for (i = 0; i < 4; i++) {
        res[i] += di_Q[i] + KRK[i] - P[i];
        assert(fabs(res[i]) < 1e-6);
    }
}

static void test_bad_args(void)
{
    double K[2];
    double Rzero[1] = { 0.0 };

    assert(lqr_gain(7, 1, di_A, di_B, di_Q, di_R, K, NULL) == LQR_EARG);
    assert(lqr_gain(2, 0, di_A, di_B, di_Q, di_R, K, NULL) == LQR_EARG);
    assert(lqr_gain(2, 1, di_A, di_B, di_Q, di_R, NULL, NULL) == LQR_EARG);
    assert(lqr_gain(2, 1, di_A, di_B, di_Q, Rzero, K, NULL) == LQR_EARG);
}

static void test_discretize(void)
{
    double Ac[4] = { 0.0, 1.0, 0.0, 0.0 };
    double Bc[2] = { 0.0, 1.0 };
    double Ad[4], Bd[2];

    lqr_discretize(2, 1, Ac, Bc, 0.1, Ad, Bd);

    assert(fabs(Ad[0] - 1.0) < 1e-12);
    assert(fabs(Ad[1] - 0.1) < 1e-12);
    assert(fabs(Ad[2] - 0.0) < 1e-12);
    assert(fabs(Ad[3] - 1.0) < 1e-12);
    assert(fabs(Bd[0] - 0.0) < 1e-12);
    assert(fabs(Bd[1] - 0.1) < 1e-12);
}

static void test_no_convergence_still_writes_K(void)
{
    double K[2] = { 0.0, 0.0 };
    lqr_opts o = { 0.0, 0 };

    o.max_iters = 1;
    assert(lqr_gain(2, 1, di_A, di_B, di_Q, di_R, K, &o) == LQR_ENOCONV);
    assert(isfinite(K[0]) && isfinite(K[1]));
}

int main(void)
{
    test_scalar_closed_form();
    test_double_integrator_settles();
    test_dare_residual();
    test_bad_args();
    test_discretize();
    test_no_convergence_still_writes_K();
    printf("all tests passed\n");
    return 0;
}
