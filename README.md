# foc

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Field-oriented control maths for a three-phase motor, in two small C files.

Phase currents and a rotor angle go in, three duty cycles come out. No timers,
no ADC, no PWM peripheral, no ESP-IDF — which is what makes it testable on a
host and portable off the ESP32.

The piece that joins hardware I already have into an actual drive:

```
esp-as5047p-encoder  ─ angle ─┐
                              ├─→  foc  ─→ esp-drv8323-gatedriver ─→ motor
phase currents  ──────────────┘   (upid runs the two current loops)
```

- **`src/foc.c` — the transforms.** Clarke, Park, their inverses, and
  space-vector modulation. Pure functions, `libm` and nothing else.
- **`src/foc_loop.c` — the current loop.** `foc_spin()`, which wires those
  together around two PID controllers. The only file that includes `upid.h`.
  Leave it out and the dependency leaves with it.

No `malloc`, no globals, no hidden state. Every struct is yours: declare it on
the stack, in a static, or inside your own struct, and hand over its address.

## Install

Copy `include/foc.h` and `src/foc.c` into your project and build them with your
other sources. For the current loops, also take `src/foc_loop.c` and both files
of [upid](https://github.com/JonasAtelier/upid). That is the whole install.

- **ESP-IDF** — drop the files into a component and list them in
  `idf_component_register(SRCS ...)`. Take `dt_s` from `esp_timer_get_time()`.
- **Arduino** — the files beside your `.ino`, or in
  `Arduino/libraries/foc/src/`.
- **Anything else** — add the `.c` files to your build. Needs C99, `<math.h>`,
  and `-lm`.

C++ can include the header directly; it carries an `extern "C"` block.

## Try it

```sh
make check    # the tests
make run      # find theta_offset on a simulated motor, then get it wrong
```

`make` expects a checkout of `upid` beside this one. Point it elsewhere with
`make UPID_DIR=/path/to/upid`.

## What it does

A three-phase motor has three currents that all change as the rotor turns, and
you cannot regulate a moving target with a PID. Field-oriented control fixes
that by changing where you stand:

```
i_a i_b i_c  ──Clarke──▶  i_alpha i_beta  ──Park──▶   i_d  i_q
 three sinusoids           two sinusoids            two constants
                                                          │
                                                    two PIDs
                                                          │
 three duty cycles  ◀──SVM──  v_alpha v_beta ◀─Park⁻¹─  v_d  v_q
```

Viewed from the rotor, the current is two numbers that hold still: `i_d` makes
flux and no torque, `i_q` makes torque and no flux. Drive `i_d` to zero and
`i_q` to whatever torque you want, and a plain PID is enough. That is the whole
idea, and `test/test_foc.c` asserts exactly that claim.

Angles are **float radians, electrical, wrapped to `[-pi, pi)`**, everywhere.
The two functions that take a *mechanical* angle say so in their names and
comments, because that is what an encoder reports.

## Use

```c
#include "foc.h"
#include "upid.h"

static struct upid pid_d, pid_q;   /* yours: foc only calls upid_spin() */
static struct foc_ctrl foc;

void motor_init(void)
{
    struct upid_cfg pid;
    struct foc_cfg cfg;

    upid_cfg_init(&pid);
    pid.kp    =  0.002f * 2000.0f;      /* kp = L w, ki = R w, w = bandwidth */
    pid.ki    =  0.15f  * 2000.0f;
    pid.o_min = -foc_v_max(24.0f);      /* never let it ask for more than */
    pid.o_max =  foc_v_max(24.0f);      /* the bus can give */
    upid_create(&pid_d, &pid);
    upid_create(&pid_q, &pid);

    foc_cfg_init(&cfg);
    cfg.pole_pairs   = 7;
    cfg.theta_offset = 0.8638f;         /* measured, per motor. See below. */
    cfg.reverse      = false;
    cfg.v_dc         = 24.0f;
    foc_create(&foc, &cfg, &pid_d, &pid_q);
}

/* from the PWM interrupt, once per control period */
void motor_tick(float ia, float ib, float ic, float theta_mech, float dt_s)
{
    struct foc_dq target = { 0.0f, 2.0f };    /* no flux, 2 A of torque */
    struct foc_abc i = { ia, ib, ic };
    struct foc_abc duty;

    if (foc_spin(&foc, target, i, theta_mech, dt_s, &duty) == FOC_EINVAL)
        return;                 /* FOC_ESAT is fine - the bus ran out, but
                                   duty is written and usable */

    pwm_set(duty.a, duty.b, duty.c);
}
```

If you only sample two phase currents, pass `ic = -(ia + ib)`. There is no
third path for the current to take.

Setting the PID output limits to `±foc_v_max(v_dc)` matters. Let them ask for
more than the bus can produce and `foc_svm()` will quietly scale the answer
back down, which is integral windup wearing a disguise.

## `theta_offset`, the number you have to measure

`foc` knows how to put current on the rotor's d and q axes. It does not know
*where those axes are*. The rotor is a magnet glued to a shaft and the encoder
is a chip glued to a board; the angle between what the encoder calls zero and
where the magnet actually points is whatever it came out as on assembly day.

**No datasheet has it, and no amount of software finds it.** You measure it
once per assembled motor and store it, alongside `pole_pairs` and `reverse`.

The procedure, in full, is in `examples/align.c` — read it before you power
anything. In outline: free the shaft, limit the current hard, apply a fixed
voltage vector at electrical angle zero, and the rotor swings round and locks
to it like a compass needle finding north. That position *is* electrical zero.
Then read the encoder back:

```c
cfg.theta_offset = 0.0f;
cfg.theta_offset = foc_elec_angle(&cfg, encoder_read_mech());
```

### When it is wrong

This is the failure that looks like broken hardware, so it is worth knowing by
sight. With the offset out by an angle `e`, the current loop still delivers
every amp you asked for. It just spends them on the wrong axis:

| offset error | torque current | current drawn |
|---|---|---|
| 0° | 100% | 100% |
| 15° | 97% | 100% |
| 45° | 71% | 100% |
| 90° | 0% | 100% |

Torque falls off as `cos(e)`. The current drawn never changes. So a motor with
a 90-degree error sits there pulling its full commanded current, getting hot,
and not turning — while the current loop reports perfect tracking, because in
the frame it believes in, the tracking *is* perfect. Nothing in software can
detect this. `make run` prints that table from a simulation.

Two more knobs sit beside it, and both are just as unguessable:

- **`pole_pairs`** — electrical revolutions per mechanical revolution. Count
  the magnets on the rotor and halve it, or turn the shaft one full turn under
  a fixed field and count the detents. 7 and 11 are common.
- **`reverse`** — true when the encoder counts the opposite way round from the
  phase order A-B-C. Get it wrong and the error grows with rotation instead of
  staying put, so the motor works at standstill and fights you the moment it
  moves.

## The functions

```c
struct foc_ab  foc_clarke(struct foc_abc x);
struct foc_abc foc_clarke_inv(struct foc_ab x);
struct foc_dq  foc_park(struct foc_ab x, float theta_e);
struct foc_ab  foc_park_inv(struct foc_dq x, float theta_e);
float          foc_wrap(float rad);
float          foc_v_max(float v_dc);
foc_sta        foc_svm(struct foc_ab v, float v_dc, struct foc_abc *duty);

foc_sta foc_cfg_init(struct foc_cfg *cfg);
foc_sta foc_create(struct foc_ctrl *c, const struct foc_cfg *cfg,
                   struct upid *pid_d, struct upid *pid_q);
float   foc_elec_angle(const struct foc_cfg *cfg, float theta_mech);
foc_sta foc_spin(struct foc_ctrl *c, struct foc_dq target, struct foc_abc i_abc,
                 float theta_mech, float dt_s, struct foc_abc *duty);
```

| function | what it does |
|---|---|
| `foc_clarke` | three phases to the stationary `alpha`-`beta` frame |
| `foc_clarke_inv` | back again; the result always sums to zero |
| `foc_park` | stationary frame to the rotor frame, given `theta_e` |
| `foc_park_inv` | back again |
| `foc_wrap` | any finite angle to `[-pi, pi)` |
| `foc_v_max` | `v_dc / sqrt(3)`, the largest undistorted vector |
| `foc_svm` | a voltage vector to three duty cycles in `[0, 1]` |
| `foc_cfg_init` | defaults into a `foc_cfg` — start here, not from zeros |
| `foc_create` | validate a config and adopt two `upid` controllers |
| `foc_elec_angle` | mechanical shaft angle to rotor electrical angle |
| `foc_spin` | one control sample: measure, rotate, regulate, modulate |

The transforms cannot fail, so they return their answer directly. Everything
else returns:

| code | meaning |
|---|---|
| `FOC_OK` | done |
| `FOC_EINVAL` | a `NULL`, a value that was not finite, a config that cannot be honoured, or a `dt_s` the PIDs rejected |
| `FOC_ESAT` | the request was longer than `foc_v_max(v_dc)` and was scaled to fit. **Not an error** — `duty` is written and is the best available answer |

Clarke here is **amplitude-invariant**: a balanced set of 10 A peak reads as 10
on the axes. The other common convention, power-invariant, differs by a factor
of about 1.22, so if a textbook disagrees with this library by exactly that,
now you know which one.

### Modulation

`foc_svm()` uses **min-max midpoint injection**: expand the vector to three
phase voltages, subtract the midpoint of the largest and smallest from all
three, then scale to duties. The shift is common-mode, so it cancels in every
phase-to-phase voltage and the motor cannot tell — but it recentres the trio in
`[0, 1]` and buys `2/sqrt(3)`, about **15.5% more** linear range than plain
sine modulation. That headroom is the entire reason SVPWM exists.

The duties are identical to classical sector-based SVPWM with symmetric dwell
times, in about five lines instead of sixty, and with no sector boundaries to
get wrong. The tests measure the 15.5% rather than taking it on trust.

When you ask for more than the bus can give, the vector is **scaled down with
its angle preserved**, not clipped per phase. Clipping shortens whichever phase
is furthest out by more than the other two, which rotates the vector and moves
the torque angle away from the one the current loop just worked out. Scaling
loses magnitude and keeps direction: less torque, still the right torque.

## Why separate

`esp-drv8323-gatedriver` configures gates. `esp-as5047p-encoder` reports an
angle. Neither knows what a motor is, and neither should. The maths between
them is not ESP-IDF's business either, so it lives here.

## What it does not do

- **No sensorless observer.** You need a real rotor angle every control period.
  Pair it with an encoder or a resolver. There is no back-EMF estimator, no
  flux observer, no high-frequency injection, and no open-loop startup ramp.
- **No field weakening.** Negative `i_d` to run past base speed is your
  business; `foc_spin` will happily apply the `target.d` you ask for, but it
  will not work out what that should be.
- **No dead-time compensation.** The gate driver inserts dead time between the
  high and low switches, and the resulting voltage error is worst at low duty
  and low current. `foc` hands you clean duties and does not know your bridge.
- **No MTPA.** Maximum-torque-per-amp needs the saliency of an interior-magnet
  rotor and a motor model to go with it. This assumes `L_d == L_q` and puts
  everything on `q`.
- **No cross-coupling decoupling.** At speed, `d` and `q` push on each other
  through `w L i`, and the two PIDs fight it rather than cancel it. Fine to a
  few thousand electrical rad/s; add feedforward if you go faster.
- **No six-step fallback.** No trapezoidal mode, no Hall-sensor commutation.
- **No hardware, at all.** No timers, no ADC, no PWM, no current sampling, no
  overcurrent trip. Numbers in, duty cycles out.
- **No velocity or position loop.** Cascade another `upid` outside this one and
  feed its output in as `target.q`.

**It has never been run on a motor.** Everything here is verified against a
host test suite and a simulated plant, and nothing more.

## Files

```
include/foc.h        the API, with comments
src/foc.c            the transforms and the modulator, no dependencies
src/foc_loop.c       the current loop, the only file that needs upid
examples/align.c     finding theta_offset, and what a wrong one costs
test/test_foc.c      the tests
```

## License

MIT. See `LICENSE`.
