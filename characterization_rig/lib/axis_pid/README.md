# Shared production controller core

`Control.c`, `Control.h`, `Output.h` are unmodified working-tree snapshots from
`analog-servo`; see `upstream.json` for source hashes and `LICENSE` for the MIT
notice. The working-tree hashes, not a possibly dirty checkout's commit alone,
identify what was compiled. The new PC tuner needs no sibling checkout.

`axis_pid.c/.h` wrap the controller with identical integer parameter validation,
1 kHz encoder-delta feedback, effort clamping and latched following-error/speed
guards in native and MCU builds. There is no RLS, dither, plant estimator or
PC tuning-search implementation here. The actual search is
`examples/axis_pc_tune.py` and never runs on STM32.

`source_id.json` hashes the five controller/bridge source files, and hashes the
sorted JSON map of those hashes into one identity. `source_id.h` exposes that
identity through HELLO. The host verifies the current files against this identity,
and refuses hardware motion if HELLO reports a different core. This is a core
source compatibility check, not authentication of the entire firmware binary.
Full firmware source and verified ELF/BIN hashes are separately archived.

If any core source changes, deliberately regenerate both identity files and
rebuild/revalidate firmware. Do not merely copy an old identity into a new build.
`tests/test_axis_pc_tune.py` checks identity consistency and exercises the real
controller's moving response and latched zero-on-failure guards.

Encoder/virtual effort telemetry does not certify ADC timing or independent
aperture/jitter. The drive's jumper enable cannot be removed by software.
