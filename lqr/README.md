# lqr

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Optimal control gains for small systems, in one C file. Describe the machine and
say what you dislike; get the numbers back.

- One header, one source file. Drop them into any C project.
- Only needs `libm`. No `malloc` — works fine on a microcontroller.
- Everything on the stack, so a control loop can call it at boot and forget it.

## What LQR gives you

You are probably here from PID. PID is one loop per thing you care about, and
the moment those things fight each other — the balancing robot has to drive
*forwards* to stop falling forwards — you are hand-tuning six numbers that all
interact, in the dark.

LQR replaces that with one question: **what do you dislike, and how much?**

You write down two weight matrices. `Q` says how much you mind each state
drifting away from zero. `R` says how much you mind using the motor. Hand both
to `lqr_gain` and you get one gain matrix `K`, and your controller is a single
line:

```c
u = -K x;      /* every loop, forever */
```

Two things you get for free, which PID never gives you:

- **It is the best possible linear controller** for the cost you wrote. Not "the
  best I found by tuning on a Friday" — provably the minimum.
- **The closed loop is stable**, guaranteed, as long as your model is right and
  the system is controllable. No tuning session ends with the robot on its face.

The catch is in the italics: *as long as your model is right*. LQR moves the work
from tuning gains to describing your machine honestly. That is usually a much
better trade, because a model you can write down is a model you can argue about.

## Try it

```sh
make check    # run the tests
make run      # design a gain for a balancing robot and watch it recover
```

To use it in your own project, copy `include/lqr.h` and `src/lqr.c`, link the
CMake target `algo::lqr`, or use this folder as an ESP-IDF component. See [Use
one library](../README.md#use-one-library).

## Describing your system

Your machine becomes two matrices. If your states are `x` (a column of numbers
like position, speed, angle) and your input is `u` (motor torque, say), then

```
x[k+1] = A x[k] + B u[k]
```

- **`A`** is what the machine does when you leave it alone. It is the interesting
  one: a falling robot has an entry here that grows.
- **`B`** is what one unit of motor does to each state.

Most people write these from continuous-time physics (`x' = Ac x + Bc u`) and
convert:

```c
lqr_discretize(n, m, Ac, Bc, 0.01, Ad, Bd);   /* 0.01 s = a 100 Hz loop */
```

All matrices are plain row-major `double` arrays — `M[row*cols + col]`. There is
no matrix type to learn.

### Q and R: what you dislike

`Q` is `n x n`, `R` is `m x m`, and in practice both are diagonal. Only the ratio
between them matters, so pin one and move the other.

| you write | in plain words |
|-----------|----------------|
| `Q[i][i]` big | I really dislike state *i* being away from zero |
| `Q[i][i]` small | state *i* can wander, I don't much care |
| `R[j][j]` big | using motor *j* is expensive — be gentle, respond slowly |
| `R[j][j]` small | motor *j* is free — hit it hard, respond fast |

So the balancing robot in `examples/balance.c` weights pitch at 40 and position
at 1: falling over is the one failure you cannot undo, whereas drifting a few
centimetres across the floor is a nuisance. Raise `R` when the answer asks for
more torque than your motors have.

A rule of thumb that gets you close: set each `Q[i][i]` to `1 / (largest value of
state i you are willing to tolerate)²`, and each `R[j][j]` to `1 / (largest
output you want from motor j)²`. Then adjust one number at a time.

## The functions

```c
lqr_gain(n, m, A, B, Q, R, K, opts);         /* the one you want */
lqr_solve(n, m, A, B, Q, R, K, P, opts);     /* same, plus the cost matrix P */
lqr_discretize(n, m, Ac, Bc, dt, Ad, Bd);    /* continuous -> discrete */
lqr_step(n, m, A, B, K, x, x_next);          /* simulate one closed-loop step */
```

| function | what it does |
|----------|--------------|
| `lqr_gain`       | `A, B, Q, R` in, gain `K` (`m x n`) out. Your control law is `u = -K x`. |
| `lqr_solve`      | the same, and also hands back `P`. `x'Px` is the remaining cost from state `x`, which is a ready-made Lyapunov function if you want to prove something. Pass `NULL` for `P` and it behaves exactly like `lqr_gain`. |
| `lqr_discretize` | forward Euler: `Ad = I + Ac dt`, `Bd = Bc dt`. Fine when `dt` is small next to the fastest thing your plant does. |
| `lqr_step`       | `x_next = (A - B K) x`. Use it to check your design settles before you trust it to hardware. `x_next` may be the same array as `x`. |

`opts` may be `NULL`, which is almost always what you want:

| field | default | what it is |
|-------|---------|------------|
| `tol`       | 1e-9   | stop when the cost matrix stops moving by this much |
| `max_iters` | 10000  | give up after this many passes |

Return codes:

| code | meaning |
|------|---------|
| `LQR_OK`      | converged, `K` is your gain |
| `LQR_EARG`    | bad size, a `NULL`, a non-finite number, `Q` or `R` not symmetric, or `R` not invertible |
| `LQR_ENOCONV` | ran out of iterations. `K` still holds the last iterate, so you can see how close it got |

## Sizes and limits

```c
#define LQR_MAX_STATES 6
#define LQR_MAX_INPUTS 3
```

These cap the stack arrays: `n` must be 1 to 6, `m` must be 1 to 3. That covers
a balancing robot, a quadrotor attitude loop, a cart-pole, an active suspension
corner. It does not cover a full six-degree-of-freedom aircraft — for that you
want a library with a Schur solver.

The solver is a fixed-point iteration on the Riccati equation. It is short and it
converges whenever a stabilising answer exists, but slowly when the closed loop
it finds is slow: the double integrator in the tests takes 1583 passes. On a
microcontroller, run it once at startup and keep `K`.

Your `Q` must be positive semi-definite and `R` positive definite. Only a
singular `R` is detected — pass a negative weight and you will get a confident
answer to a question that means nothing.

## What it does not do

- **No continuous-time solver.** Discretise first with `lqr_discretize`.
- **No output feedback.** `u = -K x` needs the *whole* state vector, every loop.
- **No observer.** Your gyro does not measure pitch angle and your encoder does
  not measure velocity, so something has to estimate them. Pair this with
  `f_madgwick` for attitude, or an EKF for the general case, and feed its output
  in as `x`.
- **No constraints.** LQR does not know your motor saturates. Check the torque
  the gain asks for, and raise `R` until it fits.
- **No integral action.** Constant disturbances leave a constant offset. Add an
  integrator state to `A` if that matters.

## Files

```
include/lqr.h         the API, with comments
src/lqr.c             the implementation
examples/balance.c    a two-wheel balancing robot catching a 5-degree tilt
test/test_lqr.c       the tests
```

## License

MIT. See `LICENSE`.
