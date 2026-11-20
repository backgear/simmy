# Single-axis characterization: a small, useful first pass

Goal: identify enough of one axis to choose control bandwidth, diagnose ringing,
and predict reversal/settling. Do not reconstruct every physical part.

For the host/MCU boundary, see the [draft USB CDC device protocol](axis_device_protocol.md):
timed command uploads, up to four encoder channels, analog effort capture,
buffered/RLE retrieval, and safety/fault semantics. Firmware is supplied separately;
the existing acquisition adapter does not yet implement this wire protocol.

## Instrumentation

Minimum: synchronized motor position, measured actuator effort, applied command,
and timestamps. Preferred: add **one temporary load-position sensor**. No encoder
on every body. Motor-only tests cannot certify load tracking or uniquely identify
internal compliance/backlash. Supply encoder scales, transmission ratios, and
geometry rather than asking a fitter to discover them.

Use measured current calibrated to torque/force to bypass amplifier command
nonlinearity in the mechanical fit. Log command too so command-to-current behavior
can be characterized separately. Saturation cannot generally be inverted. For an
AC servo use torque-producing current; for a stepper, log both phase currents and
rotor angle and derive effort from an appropriate electrical-angle/torque model.
Commanded steps and a single phase current are not mechanical effort measurements.

100 kHz capture is available, not mandatory. Resolution, synchronization, current
sensor bandwidth, and anti-alias filtering matter more. Select the analysis rate
from measured dynamics (initially aim for 10–20 samples per highest relevant
frequency). Filter before downsampling. Preserve raw recordings.

## Experiments

1. Slow bidirectional traverses at several speeds, with reversals: friction,
   directional bias, lost motion. Include dwell and repeat trials.
2. Jerk-limited acceleration/deceleration moves at several levels: inertia,
   effort limits, gross deflection.
3. Small swept-sine or multisine excitation with transmission preloaded: resonant
   frequencies, damping, delays. Record the injected signal independently of
   controller output. Repeat at multiple amplitudes to detect nonlinearity.
4. A separate ordinary reversing move: validation only, without retuning.

Use conservative travel/current/velocity limits, physical stops/estop, a known
stable controller, and a low-amplitude survey first. Host Python is not a realtime
controller. The acquisition device owns timing, limit checking, watchdogs, and
safe shutdown. Stop on saturation or unexpected motion; retain flags in the log.
A sweep can excite a dangerous resonance even at modest command amplitude.

## Model and stopping point

Start with effective inertia + viscous/Coulomb friction + constant bias. Inspect
measured frequency response and relative motor/load motion before adding one
compliant mode (two bodies), damping, or backlash. Additional bodies must explain
repeatable, observable dynamics, not merely improve training error.

A useful final artifact is both effort-to-position frequency response (motor and
load) and a compact model. Judge held-out load error, reversal delay, ringing
frequency/decay, and repeatability. Stop when these meet the application's needs.
Closed-loop identification needs care: measurement noise and feedback can bias
ordinary least squares. Use independently injected excitation and appropriate
closed-loop identification methods for a production control model.

## Prototype script

`examples/identify_axis.py` is an **offline prototype**, not a hardware driver.
Python dependencies: NumPy for planning/demo/fitting (`python3 -m pip install numpy`).
The C library remains dependency-free. Commands:

```
python3 examples/identify_axis.py plan --output build/axis_plan.csv
python3 examples/identify_axis.py demo --output build/axis_demo.csv
python3 examples/identify_axis.py fit build/axis_demo.csv --output build/axis_fit.json
python3 -m unittest discover -s tests -p 'test_axis_identification.py'
```

The plan includes separate `friction`, `acceleration`, `dynamic`, and `validation`
segments. Position targets are smooth bounded out-and-back moves, with dwell (the prototype
uses varying-speed traverses, not a full constant-speed friction protocol);
`excitation` is an additive **effort** chirp during the dynamic segment. Position
and effort use the units selected by the operator. Defaults are illustrative,
not safe machine settings. The dynamic target holds a preload offset, but this
does not guarantee that backlash remains engaged: verify measured force/motion.
The chirp increases linearly in frequency. Adjust amplitude, duration, and band
before use. Set encoder/effort scaling in the adapter, not in hidden fit factors.

To connect equipment, provide an executable adapter:

```
python3 examples/identify_axis.py capture --plan build/axis_plan.csv \
  --output build/axis_measured.csv --adapter /path/to/axis_adapter
```

The script invokes `adapter --plan ABS_PATH --output ABS_PATH`. The adapter must
explicitly implement this protocol, safely execute the plan with device-clock
scheduling, and finish acquisition before returning. No hardware execution occurs
in `plan`, `demo`, or `fit`. Capture refuses to overwrite an existing log. Check
adapter documentation and safety behavior before using capture.

Plan columns: `time,segment,target,excitation`.
Measurement columns: `time,segment,command,effort,motor_position`; optionally
`load_position` and any additional channels (phase currents, flags, etc.). Time is
seconds, strictly increasing. Segment names must be preserved; sampling need not
match the plan rate. `command` is the actual total actuator command, not the
position target. `effort` is calibrated actual torque/force, not command. All
position and effort coordinates must be mechanically consistent.

The default fitter handles the rigid baseline:

    effort = inertia * acceleration + viscous * velocity
             + coulomb * sign(velocity) + bias

It uses integral momentum balances over windows, velocity from measured position,
and ordinary least squares, excluding dynamic and validation segments. Window
length trades noise suppression against lost detail. Near-zero and reversing
windows are excluded; set `--min-speed` in position units/second to suit the
encoder resolution and axis scale (default `1e-4` is only illustrative).
It reports matrix condition,
effort-balance residuals, and held-out validation error. These are **not** a
load-position prediction or a fitted compliant chain. A nonphysical fit fails
rather than silently clamping parameters. Near-zero velocity/stiction, quantized
position, poor excitation, actuator delay, and flexible dynamics can invalidate
this baseline. Inspect residuals and vary window length; do not automatically
load its JSON into simmy or deploy a controller from it.

## Two-body compliance and backlash fitting

An optional two-body path now estimates motor/load inertia, motor/load viscous
friction and constant bias, coupling stiffness/damping, and backlash half-gap.
It requires motor **and load position**, in the same reflected mechanical
coordinate, with their relative zero calibrated. No intermediate encoder is
needed. An unknown relative encoder offset must be calibrated before fitting;
it is not silently absorbed into the gap. Nonlinear geometry is out of scope.

```
python3 -m pip install scipy matplotlib  # optional analysis dependencies
python3 examples/identify_axis.py demo --model two-body --output build/axis_two_body.csv
python3 examples/identify_axis.py fit build/axis_two_body.csv --model two-body \
  --max-gap .005 --output build/axis_two_body_fit.json \
  --predictions build/axis_two_body_predictions.csv
python3 examples/plot_axis_identification.py build/axis_two_body_predictions.csv \
  build/axis_two_body_fit.json --output build/axis_two_body.png
python3 -m unittest discover -s tests -p 'test_axis*.py'
```

**Plot location in this example: `build/axis_two_body.png`.** All paths are
explicit output arguments; the tool does not create plots automatically.

The model uses the same coupling convention as simmy:

    relative = motor_position - load_position
    spring_extension = sign(relative) * max(abs(relative) - half_gap, 0)
    coupling_force = stiffness * spring_extension + damping * relative_velocity

`backlash_half_gap` is the distance from the centered relative zero to one flank;
the full lost-motion width is twice this value. It corresponds to simmy's
`backlashGap`, despite that field's shorter name. The damper remains active
**inside the gap**, exactly as in the existing C model. This is a dead-zone spring
model, not an impact/contact solver or a general hysteresis model. Motor effort
acts only on the motor body; the load has no separate commanded force.

The fitter profiles 41 gap candidates within the operator-supplied `--max-gap`
(in position units), refines the best neighborhood, and solves a constrained
linear integral balance at each candidate. Inertias/stiffness are positive;
viscous friction/damping are nonnegative; biases may have either sign. Training
uses friction, acceleration, and dynamic segments; validation is excluded from
all parameter selection. Both signs of relative motion and engagement are
required. A gap search bound is a search limit, not an inferred physical limit.
Inspect the gap-error profile, conditioning, boundary warnings, and improvement
over the zero-gap model. A flat profile is not proof of a precisely measured gap.
Try different window lengths and search bounds, and repeat the experiment.

The two-body path currently omits Coulomb friction/stiction. If those dominate,
its fitted stiffness/gap may be biased: inspect slow-motion residuals and do not
mistake this prototype for a complete friction-plus-backlash identifier.
This path does not use the rigid fitter's `--min-speed` exclusion: reversal and
gap-crossing windows carry useful information for it.

Validation includes **open-loop trajectory replay of measured effort** for each
held-out block, with only its initial positions/velocities supplied by the
measurements. Subsequent position samples do not reset the prediction. The report
contains motor/load position RMSE and maximum error; the predictions CSV and plot
show measured versus predicted trajectories and residuals. Initial velocities
are estimated from positions; noisy starts can cause drift. Recorded effort is
linearly interpolated during replay. This is not a simulation of the controller
or proof of closed-loop stability.

Fit JSON uses physical seconds and calibrated effort/position units. Simmy's
mechanical stepping uses per-tick units; do not copy fitted coefficients into a
simmy config without an explicit time/coordinate/effort conversion.

The required next replay interface is described in the
[device contract](axis_device_protocol.md#required-host-artifact-recorded-move-replay-and-comparison):
archive the actual recording and replay any characterized move or the full run,
not a regenerated ideal trajectory. Overlay measured/simulated positions and
plot their errors. Mechanical replay uses measured effort; end-to-end command
replay requires an explicit drive/controller model. The current prototype only
replays held-out validation blocks, so arbitrary/full-run replay remains to be
implemented.

Remaining extensions: spectral estimates/coherence for injected excitation,
Coulomb/stiction in the compliant model, sensor-offset calibration, uncertainty
from repeated trials, and simmy unit conversion/export. Both demos check the
pipeline against synthetic models, not real-machine identification accuracy.
