/* DC motor speed loop: hold 1500 rpm through a load step, without retuning.
 *
 * The plant is  J*w' = Kt*u - B*w - load, so in the controller's terms
 * w' = b0*u + disturbance,  with b0 = Kt/J. Viscous friction and the load
 * torque are both left in the disturbance - the observer works them out.
 *
 * The controller is told b0 = 200 while the real plant gain is 250. Being
 * 20% wrong is on purpose: this is what a datasheet-and-a-guess looks like,
 * and ADRC absorbs it as one more disturbance to cancel. */
#include "adrc.h"
#include <stdio.h>

#define PI 3.14159265358979323846

int main(void)
{
    const double J = 2e-4;      /* rotor inertia, kg m^2      */
    const double B = 1e-4;      /* viscous friction, N m s    */
    const double Kt = 0.05;     /* torque constant, N m per A */
    const double dt = 1e-3;     /* 1 kHz control loop         */
    const double r  = 1500.0 * 2.0 * PI / 60.0;    /* setpoint, rad/s */

    adrc_cfg cfg = {
        .order = 1,             /* speed loop: one integrator, current to speed */
        .b0    = 200.0,         /* guessed Kt/J; the truth is 250 */
        .wc    = 30.0,          /* rad/s, so roughly a 1/30 s response */
        .wo    = 0.0,           /* 0 = pick 5 * wc for me */
        .u_min = -10.0,         /* the drive can source +/- 10 A */
        .u_max = 10.0
    };
    adrc s;
    double w = 0.0;             /* motor speed, rad/s */
    int k;

    if (adrc_init(&s, &cfg) != ADRC_OK) return 1;
    adrc_reset(&s, w);

    printf("  t       speed        u    disturbance est\n");
    for (k = 0; k < 2000; k++) {                   /* 2 s */
        double t    = k * dt;
        double load = (t >= 1.0) ? 0.10 : 0.0;     /* N m, applied at t = 1 s */
        double u    = adrc_update(&s, r, w, dt);

        if (k % 50 == 0)                           /* every 50 ms */
            printf("%5.2f  %7.1f rpm  %6.3f A   %9.1f\n",
                   t, w * 60.0 / (2.0 * PI), u, s.z[1]);

        w += dt * ((Kt * u - B * w - load) / J);   /* the plant */
    }

    printf("\nThe load lands at t = 1.00 with no warning and no retuning.\n"
           "Speed dips a few rpm, the estimate moves, and the current it\n"
           "takes to hold 1500 rpm goes up on its own.\n");
    return 0;
}
