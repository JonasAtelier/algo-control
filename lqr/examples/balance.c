/* Two-wheel balancing robot, linearised upright: design a gain, then watch it
 * catch a 5-degree tilt.
 *
 * States are the classic cart-pole four: cart position and speed, then body
 * pitch and pitch rate. The input is wheel torque; a torque tau at wheel
 * radius r pushes the chassis with force F = tau / r.
 *
 *   x''  = -(mb g / Mc) th + F / Mc
 *   th'' =  ((Mc + mb) g / (Mc l)) th - F / (Mc l)
 *
 * The second line is the whole problem: the th coefficient is positive, so
 * pitch runs away from upright on its own. That is what the gain is for.
 */
#include "lqr.h"
#include <stdio.h>
#include <math.h>

#define N  4          /* x, x', pitch, pitch' */
#define M  1          /* wheel torque */
#define DT 0.01       /* 100 Hz control loop */

#define DEG (180.0 / 3.14159265358979323846)

int main(void)
{
    const double g  = 9.81;
    const double Mc = 1.0;      /* chassis + wheels, kg   */
    const double mb = 0.5;      /* body above the axle, kg */
    const double l  = 0.2;      /* axle to the body centre of mass, m */
    const double rw = 0.05;     /* wheel radius, m */

    double Ac[N * N] = { 0 }, Bc[N * M] = { 0 };
    double Ad[N * N], Bd[N * M];
    double Q[N * N] = { 0 }, R[M * M], K[M * N];
    double x[N] = { 0 };
    int rc, k, i;

    Ac[0 * N + 1] = 1.0;                        /* x'   = x'            */
    Ac[1 * N + 2] = -mb * g / Mc;               /* x''  from pitch      */
    Ac[2 * N + 3] = 1.0;                        /* th'  = th'           */
    Ac[3 * N + 2] = (Mc + mb) * g / (Mc * l);   /* th'' from pitch      */

    Bc[1] =  1.0 / (Mc * rw);                   /* x''  per Nm of torque */
    Bc[3] = -1.0 / (Mc * l * rw);               /* th'' per Nm of torque */

    lqr_discretize(N, M, Ac, Bc, DT, Ad, Bd);

    /* Q charges you for each state wandering, R for using the motor.
     * Pitch (entry 2) is raised to 40 because tipping over is the only failure
     * that cannot be undone, and pitch rate (entry 3) to 4 so the answer is not
     * a twitchy one. Position and speed stay at 1: drifting a few centimetres
     * along the floor is a nuisance, not a crash. */
    Q[0 * N + 0] =  1.0;
    Q[1 * N + 1] =  1.0;
    Q[2 * N + 2] = 40.0;
    Q[3 * N + 3] =  4.0;
    R[0] = 1.0;

    rc = lqr_gain(N, M, Ad, Bd, Q, R, K, NULL);
    printf("lqr_gain rc = %d\n", rc);
    if (rc != LQR_OK) return 1;

    printf("K =");
    for (i = 0; i < N; i++) printf(" %9.4f", K[i]);
    printf("     (torque = -K x, Nm)\n\n");

    x[2] = 5.0 / DEG;                           /* released at 5 degrees */

    printf("   t(s)   pitch(deg)      x(m)   torque(Nm)\n");
    for (k = 0; k <= 300; k++) {                /* 3 seconds */
        if (k % 10 == 0) {                      /* every 100 ms */
            double u = 0.0;
            for (i = 0; i < N; i++) u -= K[i] * x[i];
            printf("  %5.2f   %10.4f  %8.4f   %10.4f\n", k * DT, x[2] * DEG, x[0], u);
        }
        lqr_step(N, M, Ad, Bd, K, x, x);
    }

    return 0;
}
