# adrc

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: GPL-3.0-only" src="https://img.shields.io/badge/license-GPL--3.0--only-f2bd68?style=flat-square&labelColor=0d2238">
</p>

A controller that works out what is fighting your plant and cancels it, in one C file.

## Why not PID

A PID controller only ever sees the error. When a load lands on the motor, the
error has to grow before the integral term slowly winds up enough current to
push back. It gets there, but it gets there late, and the gains that made that
work at 500 rpm with an empty spindle are rarely the gains that work at 3000 rpm
with a full one. So you tune again. And again, per machine, per season.

Active disturbance rejection control does the opposite. It keeps a running
estimate of **everything** that is making the plant misbehave, lumped into a
single number: the load, the friction, the temperature drift, the fact that your
model was wrong. Every step it subtracts that estimate straight back out of the
output, which leaves a plant that behaves like the simple one you assumed. You
tune that simple plant once, with two numbers, and the tuning holds while the
real world moves around underneath it.

## Try it

```sh
make check    # run the tests
make run      # run the DC motor example
```

To use it in your own project, copy `include/adrc.h` and `src/adrc.c`, link the
CMake target `algo::adrc`, or use this folder as an ESP-IDF component. See [Use
one library](../README.md#use-one-library).

The example runs a speed loop at 1500 rpm and drops a load on it at t = 1 s:

```
 0.95   1500.0 rpm   0.314 A       -62.8
 1.00   1500.0 rpm   0.314 A       -62.8
 1.05   1484.5 rpm   2.486 A      -449.4     <- load lands, estimate moves
 1.15   1499.0 rpm   2.325 A      -461.9
 1.30   1500.0 rpm   2.314 A      -462.8     <- back on target, 7x the current
```

Nothing was retuned and nothing was told about the load. The current went up
because the estimate went down.

## A complete example

```c
#include "adrc.h"

adrc_cfg cfg = {
    .order = 1,        /* speed loop */
    .b0    = 200.0,    /* how much speed change one amp buys you */
    .wc    = 30.0      /* how fast you want it, rad/s */
};
adrc s;

adrc_init(&s, &cfg);
adrc_reset(&s, read_speed());        /* start the observer where the plant is */

/* then, once per control period: */
double u = adrc_update(&s, target_speed, read_speed(), 0.001);
drive_motor(u);
```

That is the whole API in use. Everything below is about picking the numbers.

## Order: 1 or 2?

How many integrators sit between your output and your measurement?

- **`order = 1`** — you command something that sets the *rate* of what you
  measure. Current sets acceleration, you measure speed. A heater sets the rate
  of temperature rise. Most loops are this one.
- **`order = 2`** — you command something two steps removed. Current sets
  acceleration, you measure *position*. A position loop is the usual case.

Guess wrong and it will still limp along, but sluggishly. Count the integrators.

## The two knobs

| knob | what it means | where to start |
|------|---------------|----------------|
| `wc` | **controller bandwidth**, rad/s. How fast you want the plant to chase the setpoint. Raise it for a snappier response, lower it if the loop gets twitchy or the actuator saturates. | roughly `3 / (settling time you want, in seconds)` |
| `wo` | **observer bandwidth**, rad/s. How fast the disturbance estimate keeps up. Higher rejects load changes faster and amplifies sensor noise more. | leave it `0`, which picks `5 * wc` |

Tune `wc` first with `wo` left at 0. Only touch `wo` if the loop is noisy (lower
it) or slow to shrug off a load (raise it). Keep `wo` between 3 and 10 times
`wc`; outside that range one of the two is fighting the other.

There is no third gain. There is nothing to trade off against anything else.

## Picking b0

`b0` is your one piece of plant knowledge: **how much the measurement's rate
changes per unit of output**.

```
order 1:  y'  ~=  b0 * u + disturbance
order 2:  y'' ~=  b0 * u + disturbance
```

For a motor speed loop driven in amps, that is torque constant over inertia,
`Kt / J`. If you have no datasheet, measure it: from standstill, apply a step
`u`, see how fast the measurement ramps, and divide.

```
b0 ~= (change in rate) / (the u you applied)
```

Being wrong by a factor of two in either direction is fine — the error just
becomes one more disturbance for the observer to cancel. The motor example
deliberately runs with a `b0` that is 20% low. Getting the **sign** wrong is
not fine: the loop will run away. If more output makes your measurement go
down, `b0` is negative.

## The functions

| call | what it does |
|------|--------------|
| `adrc_init(&s, &cfg)` | checks the settings and starts the controller. `ADRC_OK` or `ADRC_EARG`. |
| `adrc_update(&s, setpoint, measurement, dt)` | one step. Returns the output to send, already clamped. |
| `adrc_reset(&s, measurement)` | restarts the observer where the plant is, with no disturbance and no output. |

Read `s.z[]` if you want to see the controller's thinking: `z[0]` is its
estimate of the measurement, and `z[order]` is the total disturbance it is
cancelling. In the motor example that last number is what moves when the load
lands. Everything in the struct is yours to read and nothing else.

Output limits live in the cfg as `u_min` and `u_max`. Leave **both** at 0 for no
limits. When the output does hit a limit, the observer is updated with the value
that was actually sent, not the one the controller wanted, so a saturated
actuator cannot wind up the estimate — the anti-windup is not something you
configure.

## Files

```
include/adrc.h       the API, with comments
src/adrc.c           the implementation
examples/motor.c     a DC motor speed loop taking a load step
test/test_adrc.c     the tests
```

## Limits, so you are not surprised

- The observer is discretised with **forward Euler**, so `dt` has to be small
  next to `wo`. Keep `wo * dt` under 0.2. At a 1 kHz loop that means `wo` up to
  200 rad/s, which is roomy; if you need more, sample faster.
- `dt` is yours to supply and yours to keep steady. The library has no clock.
  It is a plain multiplier here, so jitter shows up directly in the estimate.
- A `dt` that is zero, negative or not finite, and any setpoint or measurement
  that is not finite, is refused: the previous output comes back unchanged and
  the observer is left alone. One bad reading cannot poison the controller, but
  a permanently broken sensor will quietly freeze the output.
- Single input, single output. Run one controller per axis; they do not
  coordinate.
- Linear ADRC only. No nonlinear gain shaping, no tracking differentiator on the
  setpoint, so a step setpoint is asked for as a step.

## What it does not do

- No tuning for you. `b0` and `wc` are your two decisions.
- No plant model, no system identification, no state machine, no bumpless
  manual/auto handover — if you need to hand the output back and forth, call
  `adrc_reset()` before you re-enable it.
- No I/O, no clock, no allocation, no globals, no platform headers. You read the
  sensor, you supply `dt`, you drive the actuator.

## License

© 2026 JonasAtelier, GPL-3.0-only. See `LICENSE`.
