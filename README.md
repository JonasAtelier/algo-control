# algo-control

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Control for robots — four small controllers in portable C99, each one header
and one `.c` file, no heap. A setpoint and a measurement go in, an output
comes out.

## Libraries

| Library | What it does | Pick it when |
|---|---|---|
| [adrc](adrc) | Estimates whatever is fighting your plant and cancels it | PID needs retuning every time the load changes |
| [lqr](lqr) | Optimal state-feedback gains for small systems | you have a model and several coupled states — a balancing robot |
| [impedance](impedance) | Impedance and admittance control | the robot touches things and has to give way like a spring |
| [foc](foc) | Field-oriented control maths for three-phase motors | you drive a BLDC/PMSM and want torque, not just speed; needs [upid](https://github.com/JonasAtelier/upid) |

## Use one library

Every library stands on its own: no shared headers, nothing from one library
needed by another. Take the one you need and leave the rest. Three ways in.

**Copy the files.** `adrc/include/adrc.h` and `adrc/src/adrc.c` into your
project, built with your own sources. That is the whole install.

**CMake.** Every library is its own target, `algo::<folder>`. Add just the
folder you need:

```cmake
add_subdirectory(algo-control/adrc)
target_link_libraries(app PRIVATE algo::adrc)
```

or, without a copy in your tree:

```cmake
include(FetchContent)
FetchContent_Declare(algo_adrc
    GIT_REPOSITORY https://github.com/JonasAtelier/algo-control.git
    GIT_TAG        stable
    SOURCE_SUBDIR  adrc)
FetchContent_MakeAvailable(algo_adrc)
target_link_libraries(app PRIVATE algo::adrc)
```

`add_subdirectory(algo-control)`, or FetchContent without `SOURCE_SUBDIR`, adds
every library's target instead; link the ones you use. `foc` fetches upid when
it is configured, so an adrc-only project should add `adrc` alone. Examples and
tests only build when this repo is the top-level project.

**ESP-IDF.** Point `EXTRA_COMPONENT_DIRS` at one library's folder. The
component is named after the folder:

```cmake
# top-level CMakeLists.txt of your IDF project, before project()
list(APPEND EXTRA_COMPONENT_DIRS path/to/algo-control/adrc)
```

then `REQUIRES adrc` in your component's `idf_component_register()`.

Pointing it at the repo root instead gives you **one** component holding
every library, named after the folder (`algo-control`): IDF does not look
inside a folder that has its own `CMakeLists.txt`. Use one or the other,
never both — every symbol would be defined twice.

### foc and upid

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

## Build and test

```sh
make check    # every library's own tests, stopping at the first failure
make clean
```

Or with CMake and CTest. CTest runs the C tests only; the C++ linkage tests
run under `make check`.

```sh
cmake -B build && cmake --build build -j2 && ctest --test-dir build
```

## License

MIT. See [LICENSE](LICENSE).
