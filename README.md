# foc

<p align="center">
  <img alt="platform: portable C99" src="https://img.shields.io/badge/platform-portable%20C99-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="language: C" src="https://img.shields.io/badge/language-C-70e1b2?style=flat-square&labelColor=0d2238">
  <img alt="license: MIT" src="https://img.shields.io/badge/license-MIT-f2bd68?style=flat-square&labelColor=0d2238">
</p>

Field-oriented control maths for a three-phase motor, in one C file.

Placeholder — not written yet.

## What it will be

The piece that joins hardware I already have into an actual drive:

```
esp-as5047p-encoder  ─ angle ─┐
                              ├─→  foc  ─→ esp-drv8323-gatedriver ─→ motor
phase currents  ──────────────┘   (upid runs the two current loops)
```

- Clarke and Park, and their inverses.
- Space-vector modulation to three duty cycles.
- The d-q current loops, driven by `upid`.
- No hardware, no timers, no ADC. Numbers in, duty cycles out — which is what
  makes it testable on a host and portable off the ESP32.

## Why separate

`esp-drv8323-gatedriver` configures gates. `esp-as5047p-encoder` reports an
angle. Neither knows what a motor is, and neither should. The maths between
them is not ESP-IDF's business either, so it lives here.

## License

MIT
