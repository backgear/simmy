# Single-axis characterization: a small, useful first pass

Goal: identify enough of one axis to choose control bandwidth, diagnose ringing,
and predict reversal/settling. Do not reconstruct every physical part.

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

The current fitter deliberately handles only the rigid baseline:

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

Next extensions, after real recordings: spectral estimates/coherence for the
injected sweep, two-body position-domain fitting, backlash fitting, uncertainty
from repeated trials, and held-out motor/load trajectory simulation. The demo
only checks the pipeline against a synthetic rigid model; it is not evidence of
real-machine identification accuracy.
