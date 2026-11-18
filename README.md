# algo-control

Control motor speed, balance a robot or make a joint yield to contact. Each controller is written in portable C99.

**Contents** · [Choose a library](#choose-a-library) · [Try the example](#try-the-example) · [Add it to your project](#add-it-to-your-project)

## Choose a library

| Library | What it does | Example and API |
| --- | --- | --- |
| [adrc](adrc/) | Estimate the effect of an unexpected load and compensate for it. | [motor](adrc/examples/motor.c) · [API](adrc/include/adrc.h) |
| [lqr](lqr/) | Compute feedback gains from a model with several interacting variables. | [balance](lqr/examples/balance.c) · [API](lqr/include/lqr.h) |
| [impedance](impedance/) | Make a robot yield like a spring when it touches something. | [push](impedance/examples/push.c) · [API](impedance/include/imp.h) |
| [foc](foc/) | Calculate currents and voltages for field-oriented control of a three-phase motor. | [align](foc/examples/align.c) · [API](foc/include/foc.h) |

## Try the example

From this repository’s root, use Make and a C99 compiler. No robot hardware is needed.

```sh
make -C adrc run
```

Watch a simulated motor regulate its speed around 1,500 rpm as the load changes. [Example source](adrc/examples/motor.c)

## Add it to your project

For plain C, compile the source files in the library’s `src/` directory and add its `include/` directory to your compiler’s include path. For a CMake application named `app`, place this checkout inside your project and use the following lines.

```cmake
add_subdirectory(algo-control/adrc)
target_link_libraries(app PRIVATE algo::adrc)
```

For ESP-IDF, add the library directory to `EXTRA_COMPONENT_DIRS` and its name to the consuming component’s `REQUIRES`. Select either the individual libraries or the entire repository to avoid duplicate symbols.

<details>
<summary>Building FOC current loops with UPID</summary>

`foc` is the one library here with an outside dependency: its current loops
run on [upid](https://github.com/JonasAtelier/upid), a PID controller that
lives in its own repo. `foc.c` alone needs nothing; `foc_loop.c` needs upid.

- **Copying files** — also take `include/upid.h` and `src/upid.c` from upid.
- **CMake** — `algo::foc` fetches upid at a pinned commit and links it for
  you. Already have it? Define a target called `upid` before adding this
  repo, or pass `-DFETCHCONTENT_SOURCE_DIR_UPID=/path/to/upid`.
- **ESP-IDF** — upid ships no IDF `CMakeLists.txt`, so give it one in your
  project. With upid cloned to `components/upid/upid`, add
  `components/upid/CMakeLists.txt`:

  ```cmake
  idf_component_register(SRCS "upid/src/upid.c" INCLUDE_DIRS "upid/include")
  ```

  Pointing IDF at this repo root needs it too: that one component holds foc.

- **`make check`** — clones upid once into `.deps/`, at the same pinned
  commit. Use your own checkout with `make check UPID_DIR=/abs/path/to/upid`.

</details>

<details>
<summary>Fetch just this library from GitHub with CMake</summary>

```cmake
include(FetchContent)
FetchContent_Declare(algo_adrc
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-control.git
    GIT_TAG        stable
    SOURCE_SUBDIR  adrc)
FetchContent_MakeAvailable(algo_adrc)
target_link_libraries(app PRIVATE algo::adrc)
```

</details>

Run `make check` from the repository root to test the libraries. [MIT license](LICENSE)

[Browse the workshop catalog · Portable C libraries](https://github.com/JonasAtelier/jonas-atelier-libraries#portable-c-libraries)
