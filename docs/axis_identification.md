# Single-axis characterization: a small, useful first pass

Before tuning a control loop on a new axis, you want three numbers:
what bandwidth it can take, why it rings when it does, and how long it
takes to settle after a reversal. You do not need a complete physical
model of the machine to get them. This page describes the smallest
experiment that gets you there, and `examples/identify_axis.py`, a
prototype that plans the moves, fits the model, and checks the fit
against data it wasn't trained on.

For the host/MCU boundary, see the
[USB CDC device protocol](axis_device_protocol.md): timed command uploads,
encoder/effort capture, streamed lossless encoding and safety/fault semantics.
`characterization_rig/` supplies the Blue Pill firmware;
`examples/axis_adapter.py` implements the host boundary. Use the device's HELLO
capabilities rather than assuming the protocol's example limits. The current
rig has two encoders; its ADC timing is not qualified for identification.

## Instrumentation

At minimum, record synchronized motor position, measured actuator
effort, applied command, and timestamps. If you can add one temporary
sensor, put it on the load position. A motor-only encoder sees a stiff
coupling and a loose one the same way at low frequency, so it can tell
you the motor moved without telling you the load did -- it cannot
certify load tracking, and it cannot pin down internal compliance or
backlash on its own. Supply the encoder scales, transmission ratios, and
geometry you already know; asking the fitter to discover them from data
wastes degrees of freedom you don't have to spare.

Measure actuator effort, not commanded effort. Amplifier command is
rarely linear with output torque, especially near saturation, so fitting
against command folds that nonlinearity into your mechanical parameters
and your inertia estimate absorbs whatever gain error the amplifier had
that day. Calibrate current to torque/force instead, and log the raw
command separately so the command-to-current relationship can be
characterized on its own later. For an AC servo, use torque-producing
current. For a stepper, log both phase currents and rotor angle and
derive effort from the electrical-angle/torque relationship -- the
commanded step count and a single phase current are not a torque
measurement.

Choose a capture rate supported by the actual device and its timing validation.
Raise it only if your dynamics need it; resolution, synchronization,
current-sensor bandwidth and anti-alias filtering matter more than raw rate. Pick the
analysis rate from the dynamics you actually measure -- aim for 10-20
samples per highest relevant frequency, filter before you downsample,
and keep the raw recording. You can always decimate later; you cannot
recover an aliased signal.

## Experiments

1. Slow bidirectional traverses at several speeds, with reversals, dwell,
   and repeats. This surfaces friction, directional bias, and lost
   motion.
2. Jerk-limited acceleration/deceleration moves at several levels. This
   surfaces inertia, effort limits, and gross deflection.
3. A small swept-sine or multisine excitation with the transmission
   preloaded. This surfaces resonant frequencies, damping, and delay.
   Record the injected signal independently of the controller's output,
   and repeat at more than one amplitude -- if the resonance moves with
   amplitude, the system is nonlinear and a single linear model will not
   fit both.
4. A separate ordinary reversing move, held back for validation and
   never used to retune anything.

A sweep can excite a dangerous resonance even at a modest command
amplitude, so start with a low-amplitude survey and conservative
travel/current/velocity limits, physical stops or an e-stop, and a
controller you already trust. The host Python script is not a realtime
controller: the acquisition device, not the host, must own timing, limit
checking, watchdogs, and safe shutdown. If the device sees saturation or
unexpected motion, it stops and the log keeps a flag saying why.

## Model and stopping point

Start with the smallest model that could work: effective inertia plus
viscous/Coulomb friction plus a constant bias. Look at the measured
frequency response and the relative motion between motor and load before
adding anything -- a compliant second body, damping, or backlash earns
its place only by explaining a repeatable, observable feature of that
data, not by lowering training error on its own.

You're done when a compact model matches the effort-to-position
frequency response (motor and load) well enough for the application:
check held-out load error, reversal delay, and ringing frequency/decay,
and check that the fit repeats across trials. Closed-loop data makes
this harder than it looks -- measurement noise correlated with feedback
can bias ordinary least squares toward the wrong answer that still looks
like a good fit. Inject excitation independently of the control loop, and
use a closed-loop-aware identification method if the model needs to feed
a production controller.

## Prototype script

`examples/identify_axis.py` is an **offline prototype**, not a hardware
driver. It needs NumPy for planning, demo, and fitting
(`python3 -m pip install numpy`); the C library itself stays
dependency-free.

```
python3 examples/identify_axis.py plan --output build/axis_plan.csv
python3 examples/identify_axis.py demo --output build/axis_demo.csv
python3 examples/identify_axis.py fit build/axis_demo.csv --output build/axis_fit.json
python3 -m unittest discover -s tests -p 'test_axis_identification.py'
```

A plan has four segments: `friction`, `acceleration`, `dynamic`, and
`validation`. Position targets are smooth, bounded, out-and-back moves
with dwell (varying-speed traverses rather than a full constant-speed
friction protocol); `excitation` adds an additive **effort** chirp during
the dynamic segment, increasing linearly in frequency. Position and
effort are in whatever units the operator chose when setting up the
adapter -- the defaults are illustrative, not safe settings for a real
machine. The dynamic segment holds a preload offset to keep backlash
engaged, but a preload target doesn't guarantee engagement on real
hardware with friction and compliance of its own; check the measured
force/motion before trusting it. Adjust the chirp's amplitude, duration,
and band for your machine before running it, and set encoder/effort
scaling in the adapter rather than burying it in a fit factor later.

To run against real equipment, point `capture` at an executable adapter:

```
python3 examples/identify_axis.py capture --plan build/axis_plan.csv \
  --output build/axis_measured.csv --adapter /path/to/axis_adapter
```

The script invokes `adapter --plan ABS_PATH --output ABS_PATH` and waits
for it to finish. The adapter owns the rest: implementing this exact
calling convention, scheduling the move against the device's own clock,
and completing acquisition before it returns control. `plan`, `demo`, and
`fit` never touch hardware. `capture` refuses to overwrite an existing
log, so a rerun can't silently clobber the data you were about to
inspect. Read your adapter's own safety documentation before using it.

A plan file has columns `time,segment,target,excitation`. A measurement
file has `time,segment,command,effort,motor_position`, plus optionally
`load_position` and whatever else you logged (phase currents, flags).
Time is in seconds and strictly increasing; segment names must match the
plan, though the sampling rate need not. `command` is the actuator's
actual total command, `effort` is calibrated actual torque/force -- using
the position target for one or the raw command for the other quietly
breaks the fit. Keep every position and effort column in one consistent
mechanical coordinate frame.

The default fitter targets the rigid baseline:

    effort = inertia * acceleration + viscous * velocity
             + coulomb * sign(velocity) + bias

It integrates effort and momentum over windows, derives velocity from
measured position, and solves the four coefficients by ordinary least
squares over the friction and acceleration segments, holding the
validation segment out. A longer window suppresses noise; a shorter one
keeps detail -- there's no single right length, so try more than one. The
fitter drops near-zero and reversing-velocity windows (`--min-speed`, in
position units/second; the default `1e-4` is illustrative, not tuned to
your encoder) and reports matrix condition, effort-balance residuals, and
held-out validation error. A fit that comes out nonphysical fails loudly
instead of getting silently clamped into something plausible-looking.

What this baseline cannot see: stiction near zero velocity, quantization
in the position signal, weak excitation, actuator delay, and any
flexible mode. If the residuals look structured rather than like noise,
one of those is probably why. Inspect them, vary the window length, and
treat the output JSON as a diagnostic, not something to load into simmy
or a production controller directly.

## Two-body compliance and backlash fitting

If the rigid model's residuals show a resonance the first fit can't
explain, an optional two-body path estimates motor/load inertia,
motor/load viscous friction and bias, coupling stiffness/damping, and
backlash half-gap. It needs both motor and load position, in the same
reflected mechanical coordinate, with their relative zero calibrated
first -- an uncalibrated encoder offset does not get absorbed into the
gap estimate, it just produces a wrong one. Nonlinear geometry is out of
scope.

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

This example writes its plot to `build/axis_two_body.png`; every path is
an explicit argument, and nothing gets plotted unless you ask for it.

The model uses the same coupling convention as simmy itself:

    relative = motor_position - load_position
    spring_extension = sign(relative) * max(abs(relative) - half_gap, 0)
    coupling_force = stiffness * spring_extension + damping * relative_velocity

`backlash_half_gap` is the distance from the centered relative zero to
one flank, so the full lost-motion width is twice this value; it's the
same quantity as simmy's `backlashGap`, under a longer name. The damper
stays active inside the gap, matching the C model exactly -- this is a
dead-zone spring, not an impact solver or a general hysteresis model.
Motor effort acts on the motor body only; the load carries no commanded
force of its own.

The fitter profiles 41 candidate gap values within your `--max-gap`,
refines around the best one, and solves a constrained linear integral
balance at each candidate. Inertias and stiffness must come out positive;
viscous friction and damping must come out nonnegative; biases can have
either sign. Training uses the friction, acceleration, and dynamic
segments, with validation always held out of parameter selection, and
needs real excursions in both directions through the gap -- a search
bound only limits how far the fitter looks, it is not itself evidence of
a physical limit. Check the gap-error profile, the conditioning, any
boundary warnings, and how much the fit improves over a zero-gap model;
a flat profile across candidate gaps means the data didn't constrain the
gap, not that you found a precise one. If it looks flat, try other
window lengths and search bounds, or go collect more informative data.

This path doesn't yet model Coulomb friction or stiction, so if either
dominates your axis, expect the fitted stiffness and gap to absorb some
of that -- check the residuals during slow motion before trusting them.
It also skips the rigid fitter's `--min-speed` exclusion, because the
reversal and gap-crossing windows are exactly where this model earns its
keep.

Validation replays each held-out block open-loop from measured effort,
seeded only with that block's initial position and velocity -- later
measured positions never reset the simulated state, so drift shows up
instead of being masked. The report gives motor and load position RMSE
and maximum error; the predictions CSV and plot overlay measured against
predicted trajectories and their residuals. Initial velocities come from
differencing position, so a noisy start can seed drift that wasn't
really there. Recorded effort is linearly interpolated during replay.
This checks the plant model, not the controller: it is not a simulation
of closed-loop stability.

Fit JSON uses physical seconds and your calibrated effort/position
units. Simmy's own mechanical stepping runs in per-tick units, so copying
a fitted coefficient straight into a simmy config without converting
time, coordinates, and effort first will silently use the wrong scale.

The replay interface this needs next is described in the
[device contract](axis_device_protocol.md#required-host-artifact-recorded-move-replay-and-comparison):
archive the actual recording and replay any characterized move, or the
full run, rather than regenerating an idealized trajectory, then overlay
measured against simulated position and plot the error. A mechanical
replay only needs measured effort; replaying all the way from command
needs an explicit drive/controller model too. The current prototype
replays held-out validation blocks only -- arbitrary or full-run replay
is still to build.

What's left: spectral estimates/coherence for the injected excitation,
Coulomb/stiction in the compliant model, sensor-offset calibration,
uncertainty from repeated trials, and a simmy unit-conversion export.
Both demos check the pipeline against synthetic data generated from a
known model -- they confirm the pipeline works, not that it will
identify a real machine accurately.
