# characterization_rig

PlatformIO firmware for a "Blue Pill" STM32F103C8T6, implementing the
host/MCU wire protocol defined in
[`docs/axis_device_protocol.md`](../docs/axis_device_protocol.md): USB CDC,
`AXIS`-framed JSON requests/responses plus binary `DATA` blocks, and the
upload → seal → arm → start → retrieve state machine.

```
pio run                 # build
pio run -t upload        # flash via ST-Link (default; see platformio.ini for DFU)
pio device monitor       # not useful for the binary protocol itself, but
                          # confirms the board enumerates as a CDC-ACM device
pio test -e native       # host-native unit tests, no board required (see below)
```

## Testing

`crc32`, `sha256`, `json_writer`, and `run_buffer` (command/capture buffer
logic) are plain C/C++ with no Arduino or HAL dependency, so they run as
host-native unit tests under PlatformIO's `native` platform -- no board,
no hardware-in-the-loop:

```
pio test -e native
```

`main.cpp`/`axis_hw.cpp`/`protocol.cpp` are excluded from that environment
(`build_src_filter` in `platformio.ini`) since they pull in Arduino.h and
STM32 HAL headers the host toolchain doesn't have; those are only exercised
by the `bluepill_f103c8` build and real hardware. `.github/workflows/
characterization_rig.yml` runs both the native tests and the
`bluepill_f103c8` build on every push/PR touching this directory.

Not covered here (see "Not implemented" below): the protocol document's
own acceptance-test golden vectors, and anything that needs the actual
board (timing accuracy, encoder/PWM electrical behavior, ADC readings).

## What this device actually does

One commanded axis, step/dir, quadrature-command, or RC-filtered-PWM
analog effort output, 2 hardware-decoded quadrature encoders, and 2
analog input channels. HELLO reports the real numbers; nothing here is
a universal default, per the protocol's own framing.

| Signal | Pin | Note |
|---|---|---|
| STEP / quadrature A | PA8 | `output_mode: step_dir` / `quadrature_command` |
| DIR / quadrature B | PA9 | |
| PWM positive / negative | PA6 / PA7 | `output_mode: analog_effort`; TIM3 CH1/CH2, 32kHz carrier -- needs an external RC low-pass per channel (see below) |
| drive ENABLE | PA10 | polarity from manifest |
| ESTOP input | PB0 | active-low, pulled up; optional external switch |
| Encoder 0 A/B | PA0/PA1 | TIM2 hardware encoder (x4 only) |
| Encoder 1 A/B | PB6/PB7 | TIM4 hardware encoder (x4 only) |
| Analog 0 | PA4 | ADC1, 12-bit |
| Analog 1 | PA5 | ADC1, 12-bit |

TIM1 is reserved entirely as the device's real-time engine: a 200kHz
(5us) service ISR that is the tick counter, step-edge scheduler, capture
latch, and watchdog/estop/travel-limit check -- see the file header in
`src/axis_hw.cpp` for exactly how each piece works and its honest timing
error bound (`HELLO.max_timing_error_ticks`).

### Analog effort output (`output_mode: analog_effort`)

This part has no on-chip DAC at all, so `analog_effort` is a PWM "poor-man's
DAC": the same approach, same TIM3/PA6-PA7 pin choice, and same bipolar
-split-into-two-unipolar-channels encoding as the sibling
`~/analog-servo-f103` firmware's `Output.c`/`tim_f1.c` for this same part.
Each pin needs its own single-pole RC low-pass (e.g. ~1k/100nF, corner well
below the 32kHz carrier) feeding a downstream differential-input stage;
"zero effort" is both channels at 0% duty (both filtered outputs at their
own zero), not mid-scale. Command samples in this mode are the *held*
signed code for that interval (zero-order hold at the interval's start,
scaled by the manifest's `analog_output_full_scale`), not a cumulative
digital count -- unlike `step_dir`/`quadrature_command`, there's no
edge-rate feasibility check at `SEAL` for this mode.

## Deliberate simplifications (all surfaced honestly in HELLO/errors, not silent)

- **Only 2 encoders, not "up to 4".** TIM3's encoder-capable pins are
  spent on PWM analog output instead (see above), leaving TIM2 and TIM4
  free for quadrature decode.
- **One encoder decode mode.** Only x4 (hardware `TI12`) is wired up;
  requesting x1/x2 is `UNSUPPORTED` rather than silently promoted.
- **Manifest canonicalization is this firmware's own normalized
  reconstruction** of the fields it accepts (fixed key order, integers
  only -- a float anywhere in the manifest is `UNSUPPORTED`), not a
  generic RFC 8785 canonicalizer. `GET_RUN` echoes exactly that
  normalized form, so a host that wants to re-verify `manifest_hash`
  should canonicalize *that*, not its original request bytes. See
  `build_canonical_manifest()` in `src/protocol.cpp`.
- **SEAL requires at most one emitted step/quadrature edge per service
  tick** in every command interval (checked as a feasibility condition,
  not discovered at runtime), so the ISR never needs a catch-up loop.
  Declared via `HELLO.service_tick_hz`.
- **Declared capacities are intentionally modest** (`MAX_COMMAND_SAMPLES`,
  `MAX_CAPTURE_SAMPLES` in `include/axis_config.h`): the C8 variant has
  only 20KB of SRAM total, shared with the USB CDC stack, ArduinoJson's
  parse buffer, and the call stack. `pio run` reports ~80% RAM / ~87%
  flash used at these settings; the `bluepill_f103c8_128k` board (same
  RAM, more flash) is available if flash becomes the constraint instead.

## Not implemented / explicitly out of scope here

This firmware does not attempt the acceptance-test golden vectors the
protocol document calls for (CRC/framing fuzzing, RLE reconstruction
cross-checks, corrupt/reordered block handling, logic-analyzer timing
verification, etc.) -- those are the "implementation and interoperability
tests...still required before hardware characterization is automatic"
that the protocol document itself says are a separate, necessary step.
