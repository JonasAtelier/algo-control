/* lqr.h - discrete-time LQR gains for small state-space systems.
 *
 * C99, no dependencies beyond libm, no malloc, no globals.
 * Link with -lm.
 *
 * Every matrix is a plain row-major double array: M[row*cols + col].
 */
#ifndef LQR_H
#define LQR_H

#ifdef __cplusplus
extern "C" {
#endif

#define LQR_MAX_STATES 6
#define LQR_MAX_INPUTS 3

/* Return codes. */
#define LQR_OK       0
#define LQR_EARG    (-1)   /* bad size, NULL, non-finite, R not invertible */
#define LQR_ENOCONV (-2)   /* the Riccati iteration did not settle in max_iters;
                              K still holds the last iterate */

/* Riccati tuning. Either field left 0 uses its default. */
typedef struct {
    double tol;        /* stop when max |P_k+1 - P_k| < tol; 0 selects 1e-9 */
    int    max_iters;  /* 0 selects 10000 */
} lqr_opts;

/* Discrete system x[k+1] = A x[k] + B u[k]. Minimises sum x'Qx + u'Ru.
 * A is n x n, B is n x m, Q is n x n, R is m x m, K is m x n, all row-major.
 * Control law: u = -K x. opts may be NULL. */
int lqr_gain(int n, int m, const double *A, const double *B,
             const double *Q, const double *R, double *K, const lqr_opts *opts);

/* Same, and also returns the Riccati solution P (n x n) for cost checks. P may be NULL. */
int lqr_solve(int n, int m, const double *A, const double *B,
              const double *Q, const double *R, double *K, double *P, const lqr_opts *opts);

/* Continuous x' = Ac x + Bc u  ->  discrete at step dt. Forward Euler:
 * Ad = I + Ac dt, Bd = Bc dt. Good enough when dt is small next to the
 * plant's fastest time constant, which a 100 Hz balancing loop is. */
void lqr_discretize(int n, int m, const double *Ac, const double *Bc, double dt,
                    double *Ad, double *Bd);

/* Closed-loop step: x_next = (A - B K) x. Handy for simulating your design.
 * x_next may alias x. */
void lqr_step(int n, int m, const double *A, const double *B, const double *K,
              const double *x, double *x_next);

#ifdef __cplusplus
}
#endif
#endif /* LQR_H */
