# imp

Impedance and admittance control in one C file — the thing that makes a robot
go soft.

A normal position controller fights you: push the arm and it shoves back with
everything it has. Impedance control makes it behave like a **spring with a
shock absorber** instead. Push it, it gives way. Let go, it comes back. Push
harder, it moves further. That is what makes hand-guiding, contact tasks and
safe collaboration possible.

You pick three numbers per axis — how heavy it feels, how springy, how
sluggish — and this library turns them into commands.

- One header, one source file. Drop them into any C project.
- Only needs `libm`. No `malloc` — fits a microcontroller control loop.
- Any number of axes, any meaning: Cartesian XYZ, joints, a single slide.

## Which of the two do you need?

| your robot takes | use | you give it | you get back |
|---|---|---|---|
| **position** commands (most arms, UR/Franka in position mode, any servo stage) | `imp_admittance` | measured force | how far to move |
| **torque** commands (torque-controlled arms, direct-drive, your own motor loop) | `imp_impedance` | measured position | how much force to apply |

Admittance is what most people want, because most robots take positions. If you
are unsure, you want admittance.

## Try it

```sh
make check    # run the tests
make run      # push a simulated axis and watch it settle
```

## A complete example

One axis. A force arrives from your sensor; you add the resulting offset to
whatever position you were going to command anyway.

```c
#include "imp.h"
#include <stdio.h>

int main(void)
{
    /* heavy-ish, springy, well damped */
    imp s = { .n = 1, .axis = {{ .m = 2.0, .k = 100.0, .d = 28.3 }} };
    double desired = 0.500;      /* where you wanted the arm to be */
    int i;

    for (i = 0; i < 2000; i++) {
        double force = 10.0;                     /* pretend force sensor */

        imp_admittance(&s, &force, 0.001);       /* 0.001 s = 1 kHz loop */

        /* now command:  desired + s.offset[0]  to the robot */
    }

    printf("offset %.3f m, so command %.3f m\n", s.offset[0], desired + s.offset[0]);
    return 0;
}
```

Under a steady 10 N push against 100 N/m of stiffness, the arm settles 0.1 m
away — force divided by stiffness, exactly like a real spring. Release the
force and it returns to `desired` on its own.

That is the whole pattern: **call it every cycle, add `offset` to your target.**

## The three numbers

```c
s.axis[0].m = 2.0;      /* virtual mass      */
s.axis[0].k = 100.0;    /* virtual stiffness */
s.axis[0].d = 28.3;     /* virtual damping   */
```

- **`k` — stiffness (N/m).** How hard it resists. High = stiff and precise,
  low = soft and easy to push. This is the one you tune first.
  Deflection under a steady push is just `force / k`.
- **`m` — mass (kg).** How heavy it feels. Small = darts away from a nudge,
  large = ponderous. Start around the real mass you are moving.
- **`d` — damping (Ns/m).** How much it swallows the wobble. Too little and it
  bounces; too much and it feels like syrup.

Damping is the awkward one, so let the library work it out:

```c
s.axis[0].d = imp_damping(m, k, 1.0);   /* 1.0 = no overshoot at all */
```

`zeta` of `1.0` is critically damped — the usual choice. Below `1.0` it springs
back and wobbles, above it is sluggish. `make run` shows both.

Set `m = 0` for a **pure damper**: no springiness, no stored energy, it just
moves while you push and stops when you stop. That is the classic hand-guiding
feel (leave `k = 0` too, so it stays where you left it).

## Keeping it safe

Real force sensors are noisy and drift, and a spike should never launch your
arm. Four limits, all optional — set any to `0` to disable:

```c
s.axis[0].f_deadband = 1.0;    /* ignore anything under 1 N (sensor noise) */
s.axis[0].f_max      = 50.0;   /* never react to more than 50 N */
s.axis[0].x_max      = 0.10;   /* never deflect more than 10 cm */
s.axis[0].v_max      = 0.25;   /* never yield faster than 0.25 m/s */
```

Use the deadband. Without one, sensor noise makes the arm wander off on its
own. `x_max` and `v_max` are what stop a bad reading from becoming a fast,
large, unexpected move.

## Several axes

Axes are independent — set `n` and fill in as many as you use:

```c
imp s = { 0 };
s.n = 3;                       /* x, y, z */
for (int i = 0; i < 3; i++) {
    s.axis[i].m = 2.0;
    s.axis[i].k = 100.0;
    s.axis[i].d = imp_damping(2.0, 100.0, 1.0);
}

double force[3] = { fx, fy, fz };
imp_admittance(&s, force, dt);
/* command:  desired[i] + s.offset[i]  */
```

Different stiffness per axis is the useful trick: stiff sideways, soft along
the push direction, and a peg finds its hole by itself.

## The functions

```c
imp_admittance(&s, force, dt);                     /* force in, motion out */
imp_impedance(&s, x, v, xd, vd, force_out);        /* motion in, force out */
imp_reset(&s);                                     /* clear the deflection */
imp_damping(m, k, zeta);                           /* pick d for you */
```

`imp_impedance` computes `k * (xd - x) + d * (vd - v)` and clamps it to
`f_max`. Pass `NULL` for `v` or `vd` to mean zero.

Call `imp_reset` before you re-enable the controller after a pause — otherwise
it resumes with a stale offset and the arm jolts.

Both return `IMP_OK`, or `IMP_EARG` if something is wrong (`dt` not positive,
a negative gain, `n` out of range).

## Files

```
include/imp.h        the API, with comments
src/imp.c            the implementation
examples/push.c      simulated push-and-release, damped two ways
test/test_imp.c      the tests
```

## Limits, so you are not surprised

- **This moves a real robot.** Try new gains slowly, with `x_max`, `v_max` and
  an e-stop in reach.
- Call it at a steady, fast rate — 100 Hz to 1 kHz. `dt` must be much smaller
  than `sqrt(m / k)`, or the simulated spring goes unstable and runs away.
  With `m = 2`, `k = 100` that time constant is 0.14 s, so 1 ms is plenty.
- Axes are decoupled: separate `m`, `d`, `k` per axis, no cross terms. Fine for
  Cartesian and joint control; not the full matrix form.
- No filtering of your force input. Filter the sensor yourself, or lean on the
  deadband.
- Very stiff `k` against a very stiff environment can chatter. That is contact
  instability, not a bug — lower `k` or raise `d`.
- No gravity or friction compensation. Feed it forces that already have those
  removed, or it will drift.
