/* Push a virtual 1-axis arm for half a second, then let go and watch it
 * settle. No hardware needed - the "robot" is just the controller's own
 * response to a force you invent. */
#include "imp.h"
#include <stdio.h>

#define DT   0.001      /* 1 kHz control loop */
#define PUSH 20.0       /* newtons, applied for the first 0.5 s */

static void run(const char *label, double zeta)
{
    const double m = 2.0, k = 100.0;
    imp s = { 0 };
    int step;

    s.n = 1;
    s.axis[0].m = m;
    s.axis[0].k = k;
    s.axis[0].d = imp_damping(m, k, zeta);
    s.axis[0].f_deadband = 0.5;      /* ignore sensor noise below 0.5 N */

    printf("\n%s (zeta = %.1f, damping = %.1f)\n", label, zeta, s.axis[0].d);
    printf("  time    force   offset\n");

    for (step = 0; step <= 1500; step++) {
        double f = (step * DT < 0.5) ? PUSH : 0.0;

        imp_admittance(&s, &f, DT);

        if (step % 100 == 0) {
            int bar = (int)(s.offset[0] * 100.0 + 0.5), i;
            printf("  %4.2fs  %5.1fN  %6.3fm  ", step * DT, f, s.offset[0]);
            for (i = 0; i < bar; i++) putchar('#');
            putchar('\n');
        }
    }
}

int main(void)
{
    puts("Command  desired_position + s.offset[0]  to the robot each cycle.");
    puts("Steady offset under a steady push is force / stiffness,");
    puts("here (20 N - 0.5 N deadband) / 100 N/m = 0.195 m.");

    run("critically damped - no overshoot", 1.0);
    run("springy - overshoots and settles", 0.3);
    return 0;
}
