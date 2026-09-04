#include "lqr.h"
#include <math.h>
#include <string.h>

/* ---- small matrix helpers -------------------------------------------
 *
 * Row-major throughout: M[row*cols + col]. The destination must not be one
 * of the sources.
 */

/* C(ra x cb) = A(ra x ca) * B(ca x cb) */
static void mat_mul(int ra, int ca, int cb, const double *A, const double *B, double *C)
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
static void mat_mul_at(int ra, int ca, int cb, const double *A, const double *B, double *C)
{
    int i, j, k;
    for (i = 0; i < ca; i++)
        for (j = 0; j < cb; j++) {
            double s = 0.0;
            for (k = 0; k < ra; k++) s += A[k * ca + i] * B[k * cb + j];
            C[i * cb + j] = s;
        }
}

/* Gauss-Jordan with partial pivoting: X <- M^-1 X over nrhs columns.
 * M (m x m) and X (m x nrhs) are both destroyed. nrhs may be 0, which just
 * asks whether M is invertible. X is untouched in that case and may be NULL. */
static int gj_solve(int m, int nrhs, double *M, double *X)
{
    int i, j, k;

    for (k = 0; k < m; k++) {
        int piv = k;
        double max = fabs(M[k * m + k]), d;

        for (i = k + 1; i < m; i++)
            if (fabs(M[i * m + k]) > max) { max = fabs(M[i * m + k]); piv = i; }
        if (max < 1e-12) return LQR_EARG;                  /* singular */

        if (piv != k) {
            for (j = 0; j < m; j++) {
                double t = M[k * m + j]; M[k * m + j] = M[piv * m + j]; M[piv * m + j] = t;
            }
            for (j = 0; j < nrhs; j++) {
                double t = X[k * nrhs + j]; X[k * nrhs + j] = X[piv * nrhs + j]; X[piv * nrhs + j] = t;
            }
        }

        d = M[k * m + k];
        for (j = 0; j < m; j++)    M[k * m + j]    /= d;
        for (j = 0; j < nrhs; j++) X[k * nrhs + j] /= d;

        for (i = 0; i < m; i++) {
            double f = M[i * m + k];
            if (i == k || f == 0.0) continue;
            for (j = 0; j < m; j++)    M[i * m + j]    -= f * M[k * m + j];
            for (j = 0; j < nrhs; j++) X[i * nrhs + j] -= f * X[k * nrhs + j];
        }
    }
    return LQR_OK;
}

static int all_finite(const double *M, int count)
{
    int i;
    for (i = 0; i < count; i++)
        if (!isfinite(M[i])) return 0;
    return 1;
}

static int symmetric(const double *M, int n)
{
    int i, j;
    for (i = 0; i < n; i++)
        for (j = i + 1; j < n; j++)
            if (fabs(M[i * n + j] - M[j * n + i]) > 1e-9) return 0;
    return 1;
}

/* ---- the Riccati solver ---------------------------------------------- */

int lqr_solve(int n, int m, const double *A, const double *B,
              const double *Q, const double *R, double *K, double *P, const lqr_opts *opts)
{
    double Pk[LQR_MAX_STATES * LQR_MAX_STATES];
    double Pn[LQR_MAX_STATES * LQR_MAX_STATES];
    double PA[LQR_MAX_STATES * LQR_MAX_STATES];
    double PB[LQR_MAX_STATES * LQR_MAX_INPUTS];
    double N [LQR_MAX_INPUTS * LQR_MAX_STATES];
    double W [LQR_MAX_INPUTS * LQR_MAX_INPUTS];
    double tol;
    int max_iters, iter, i, j, converged = 0;

    if (n < 1 || n > LQR_MAX_STATES || m < 1 || m > LQR_MAX_INPUTS) return LQR_EARG;
    if (!A || !B || !Q || !R || !K) return LQR_EARG;
    if (!all_finite(A, n * n) || !all_finite(B, n * m) ||
        !all_finite(Q, n * n) || !all_finite(R, m * m)) return LQR_EARG;
    if (!symmetric(Q, n) || !symmetric(R, m)) return LQR_EARG;

    /* R has to be invertible on its own. Leaving it to the pivot check on
     * (R + B'PB) is not enough: B'PB can prop up a singular R and hand back a
     * gain for a cost that never charges for using the motor. */
    memcpy(W, R, (size_t)(m * m) * sizeof(double));
    if (gj_solve(m, 0, W, NULL) != LQR_OK) return LQR_EARG;

    tol       = (opts && opts->tol       > 0.0) ? opts->tol       : 1e-9;
    max_iters = (opts && opts->max_iters > 0)   ? opts->max_iters : 10000;

    memcpy(Pk, Q, (size_t)(n * n) * sizeof(double));      /* start from P = Q */

    /* ponytail: fixed-point iteration; swap in a Schur/doubling solver if
     * 10000 iterations is ever too slow */
    for (iter = 0; iter < max_iters && !converged; iter++) {
        double diff = 0.0;

        mat_mul   (n, n, n, Pk, A, PA);       /* PA = P A                    */
        mat_mul   (n, n, m, Pk, B, PB);       /* PB = P B                    */
        mat_mul_at(n, m, n, B, PA, N);        /* N  = B'P A          (m x n) */
        mat_mul_at(n, m, m, B, PB, W);        /* W  = B'P B          (m x m) */
        for (i = 0; i < m * m; i++) W[i] += R[i];

        /* K = (R + B'PB)^-1 B'PA, solved in place: the inverse is never formed.
         * Written every pass, so K holds the last iterate even on LQR_ENOCONV. */
        memcpy(K, N, (size_t)(m * n) * sizeof(double));
        if (gj_solve(m, n, W, K) != LQR_OK) return LQR_EARG;

        mat_mul_at(n, n, n, A, PA, Pn);       /* Pn = A'P A                  */
        mat_mul_at(m, n, n, N, K, PA);        /* PA reused: N'K = A'PB (R+B'PB)^-1 B'PA */
        for (i = 0; i < n * n; i++) Pn[i] += Q[i] - PA[i];

        /* Symmetrise. P is symmetric in exact arithmetic; forcing it stops
         * rounding from drifting the two halves apart over thousands of passes. */
        for (i = 0; i < n; i++)
            for (j = i; j < n; j++) {
                double s = 0.5 * (Pn[i * n + j] + Pn[j * n + i]);
                double d = fabs(s - Pk[i * n + j]);
                if (d > diff) diff = d;
                Pn[i * n + j] = Pn[j * n + i] = s;
            }

        memcpy(Pk, Pn, (size_t)(n * n) * sizeof(double));
        converged = (diff < tol);
    }

    if (P) memcpy(P, Pk, (size_t)(n * n) * sizeof(double));
    return converged ? LQR_OK : LQR_ENOCONV;
}

int lqr_gain(int n, int m, const double *A, const double *B,
             const double *Q, const double *R, double *K, const lqr_opts *opts)
{
    return lqr_solve(n, m, A, B, Q, R, K, NULL, opts);
}

/* ---- helpers around the solver --------------------------------------- */

void lqr_discretize(int n, int m, const double *Ac, const double *Bc, double dt,
                    double *Ad, double *Bd)
{
    int i, j;

    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            Ad[i * n + j] = Ac[i * n + j] * dt + (i == j ? 1.0 : 0.0);

    for (i = 0; i < n * m; i++) Bd[i] = Bc[i] * dt;
}

void lqr_step(int n, int m, const double *A, const double *B, const double *K,
              const double *x, double *x_next)
{
    double u[LQR_MAX_INPUTS];
    double xn[LQR_MAX_STATES];
    int i, j;

    for (i = 0; i < m; i++) {                     /* u = -K x */
        double s = 0.0;
        for (j = 0; j < n; j++) s += K[i * n + j] * x[j];
        u[i] = -s;
    }
    for (i = 0; i < n; i++) {                     /* x_next = A x + B u */
        double s = 0.0;
        for (j = 0; j < n; j++) s += A[i * n + j] * x[j];
        for (j = 0; j < m; j++) s += B[i * m + j] * u[j];
        xn[i] = s;
    }
    memcpy(x_next, xn, (size_t)n * sizeof(double));   /* via xn so x_next may alias x */
}
