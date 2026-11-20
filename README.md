# simmy

A small mechatronics simulation library: a serial chain of rigid bodies
connected by compliant (spring + damper + backlash) couplings, a
composable "analog signal chain" standing in for a real DAC/amplifier
signal path, and a bidirectional pulse source for step/dir-style
position commands. Coefficients load from JSON. No opinion on control
algorithms -- you drive it with a plain function pointer, which may be a
three-line demo P loop or a real production control cascade linked in
from somewhere else.

It grew out of a servo-firmware project's native unit tests (started as
two header-only toy models used to ask "does the control loop track a
synthetic pulse train / an accelerating one / one with injected
jitter"), then kept growing (stiction, a localized friction "speed
bump", periodic torque ripple, a two/three-mass compliant coupling with
backlash) until it made sense to pull out as its own library.

**Nothing here is a claim about any specific real machine.** Every
numeric model is intentionally simplified (see "Scope and non-goals"
below), and the example configs are representative order-of-magnitude
ratios for recognizable mechanisms, not measurements.

## Build

```
make lib        # build/libsimmy.a
make test       # plain assert()-based test suite
make examples   # three self-contained example programs (see below)
make integration  # OPTIONAL -- only if a ../analog-servo sibling repo exists
```

Only dependencies: a C compiler and libm. No Python, no third-party
JSON library -- the config parser is hand-rolled (see "JSON config
files" below).

## Scope and non-goals

- This is a **mechanical + analog-signal-path** simulator. It has no
  opinion about control algorithms. You drive it with `SimmyControllerFn`,
  a plain function pointer -- see "Controller convention" below. A demo
  P/PD loop and a real production control cascade linked in from
  elsewhere are both ordinary implementations of that same typedef;
  there is no special "simple" calling convention and no separate "real"
  one.
- Closing the control loop is assumed to happen against **one body** in
  the chain, `SimmyChainParams.feedbackBody` -- intended to be the body a
  real motor-mounted encoder would read. Reading position or velocity off
  any **other** body (e.g. a table at the far end of a lead screw,
  standing in for where a linear scale would sit) is fully supported for
  **measurement/evaluation** -- precisely so you can judge whether
  motor-side-only control tracks acceptably at the load, which is the
  comparison that matters for judging whether a config fits a real
  physical setup. Simmy does not support closing the loop against a
  non-feedback body; that's a deliberate scope boundary.
- Cogging and commutation ripple are both represented by the same plain
  sinusoidal position-dependent term (`rippleAmplitude`/
  `ripplePeriodCounts`) -- not a magnetic/commutation model, a stand-in,
  with period/amplitude picking which real effect it represents. This is
  useful as a simple position-only disturbance, but it cannot represent
  two-phase stepper torque production. The dedicated stepper model below
  combines two phase currents with rotor electrical angle; use it for
  microstepping rather than treating all motor errors as additive ripple.
- The stiction model is the textbook "decide before integrating whether
  the driving force can overcome static friction" formulation -- it
  avoids the classic bug where a naive post-hoc check makes velocity
  chatter around zero, but it's not a claim about any particular real
  friction surface.
- **Off-axis spring effects are explicitly out of scope** -- e.g. a
  preload spring or misalignment that would cause a position-dependent
  NORMAL force and hence a continuously-varying friction profile. This
  is distinct from `frictionBump*` (a flat localized patch, not a
  smoothly spring-coupled effect) and is a deliberate exclusion, not a
  gap to fill later.
- Not yet modeled (future work): **endstops/hard limits** -- position
  thresholds that produce a contact event (not just a spring, a
  one-sided hard stop), for studying home-overshoot at different
  approach velocities and deceleration/smoothing profiles.
- The stepper model is **low-speed and current-driven**, not a winding or
  chopper simulation. Winding inductance, back-EMF, voltage limits,
  saturation, and non-sinusoidal per-phase torque shapes remain future
  work. It cannot predict the high-speed torque envelope.
- Test suite is currently plain `assert()`-based; GUnit (C++ on top of
  Google Test) was suggested as a possible upgrade for a future cycle --
  would add a C++ toolchain + Google Test dependency, which cuts against
  the "C compiler + libm only" promise above, so not adopted yet.

## The mechanical body chain

A chain of 1 to `SIMMY_MAX_BODIES` (8) rigid bodies, each with its own
inertia, viscous friction, static friction (stiction + Coulomb), gravity
bias, periodic ripple, and an optional one-off localized friction "speed
bump" (a dusty/less-greased section, distinct from the recurring
ripple). Adjacent bodies connect through a `SimmyCouplingParams`:
spring stiffness, damping, and an optional backlash dead-zone.

A **1-body chain is exactly a rigid coupling** -- no spring math at all,
which is both physically correct for a truly rigid connection and
numerically the safest choice (no stiff-spring instability to worry
about). 2- and 3-body chains sub-step the coupling dynamics internally
(`SimmyChainParams.subSteps`, default 20) for numerical stability against
a stiff spring -- a realistic coupling stiffness at a 1kHz-ish tick is a
stiff ODE for plain Euler integration.

```c
SimmyChainParams params;
SimmyChainParamsFromJsonFile("my_config.json", &params, NULL, 0);
SimmyChainState state;
SimmyChainInit(&state, &params);

for (;;) {
  int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
  int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
  int32_t torque = my_controller(NULL, target, position, velocity, dtCycles);
  double force = SimmyAnalogChainStep(&analogState, &analogParams, (double)torque);
  SimmyChainStep(&state, &params, force);
}
```

## The analog signal chain

A composable "train of analog type values" standing in for a real
DAC -> amplifier -> motor signal path: an ordered list of simple stages
(`gain`, `saturate`, `quantize` -- a DAC's LSB step, `lag` -- one-pole
electrical bandwidth, `noise`), each transforming a single scalar value.
Nothing requires using all of these or in any particular order --
compose whatever path a given drive's own signal chain actually has.
This is why mechanical bodies have no `ampAlpha`-style field of their
own: electrical/analog character lives here, separate from mechanical
properties.

## Two-phase stepper: velocity-ripple milestone

The first stepper model targets disturbances to constant velocity. It
provides three independently composable parts:

- `SimmyStepperMicrostepAngle(index, M)`: signed step/dir index to
  electrical command angle, M microsteps per full step. Four full steps
  make one electrical revolution. The angle is wrapped, not the rotor.
- `SimmyStepperCurrents(phi, peakCurrent, &path, currents)`: sine/cosine
  reference currents with independently disableable per-phase DAC
  quantization, fractional gain errors, and additive offsets. Zeroed
  `SimmyStepperCurrentParams` is ideal. Quantization precedes gain/offset;
  there is no implicit current clamp. Two existing analog chains may be
  added afterward for approximate per-phase dynamics/noise.
- `SimmyStepperTorque(&motor, theta, Ia, Ib)`: torque in Nm from actual
  phase currents (A) and mechanical rotor angle (radians):

```
thetaE = electricalCyclesPerRev * theta + electricalOffset
torque = Kt * (-Ia*sin(thetaE) + Ib*cos(thetaE))
         - detentAmplitude * sin(detentCyclesPerRev*theta + detentPhase)
```

For currents `(I*cos(phi), I*sin(phi))`, the powered term is
`Kt*I*sin(phi-thetaE)`. Positive displacement from alignment produces a
restoring torque. A conventional 1.8-degree motor has 50 electrical
cycles/rev, not 200. `I` is vector peak current, not per-phase RMS;
datasheet holding-torque/current conventions need checking before fitting
`Kt`. Detent periodicity is separate and must be specified, not inferred
from the full-step count. Detent persists with zero actual currents;
phase-current errors need not.

`SimmyStepperChainStep` applies torque to body 0 and reevaluates it from
rotor position at **every mechanical substep**, even for a one-body chain.
Currents are held constant over the outer tick. It uses `subSteps` (zero
means 20); the original `SimmyChainStep` behavior is unchanged. Do not
also enable body ripple for the same detent effect.

### Units and numerical resolution

The motor uses SI units; the chain still uses counts/tick. Supply explicit
`radiansPerCount` and `forcePerNm` conversion factors. For example, with
`C` counts/radian, tick duration `dt` seconds, and chain force unit Nm
(`forcePerNm=1`):

```
position = C * theta            velocity = C * dt * omega
body.inertia = J / (C * dt^2)   body.friction = B / (C * dt)
radiansPerCount = 1 / C         body.gravityBias = load torque in Nm
```

Here `J` is kg m² and `B` is Nm/(rad/s). Couplings likewise need conversion
if starting from SI parameters. The chain's existing stiction velocity
threshold is in counts/tick, not rad/s; choose the scale deliberately.

Parameters follow the library's caller-validates convention: positive
inertia, positive conversion factors, positive electrical cycle count,
and positive M for the microstep-angle helper. Disabled current
quantization uses step zero. No automatic timestep selection is provided.
Resolve both magnetic/mechanical resonance **and command pulse timing**;
mechanical substeps cannot recover pulse edges hidden inside an outer
tick. Check convergence when changing tick rate or stiffness.

### Reproduce the plots

```
make test examples
build/stepper_velocity > build/stepper_velocity.csv
python3 examples/plot_stepper.py build/stepper_velocity.csv build/stepper
```

Only plotting requires Python, NumPy, and Matplotlib. Outputs are
`build/stepper_velocity.{png,svg}` and `build/stepper_spectrum.{png,svg}`;
the CSV includes position, velocity, load angle, actual currents, and torque.

The example compares continuous ideal commands, 16/64 microsteps per full
step, detent only, current errors only, and combined disturbances. It uses
an illustrative 200-full-step/rev motor, 1 A vector current, Kt=0.4 Nm/A,
J=4e-4 kg m² including reflected load, B=.008 Nm/(rad/s), .03 Nm load,
and .2 rad/s (1.91 rpm) target. Detent amplitude is .012 Nm at 200
cycles/rev. Current errors are +3%/-3% gain, +.01/-.015 A offset, and
1/31 A DAC increments. There is **no velocity controller** in this example.

Over the 2–4 s measurement window, RMS velocity error relative to target
is approximately:

| Scenario | RMS error / target |
|---|---:|
| Continuous ideal | 0.004% |
| 16 microsteps | 20.45% |
| 64 microsteps | 3.04% |
| Detent only, continuous command | 8.83% |
| Current errors only, continuous command | 6.08% |
| Combined, 16 microsteps | 59.17% |

The combined case briefly reverses despite positive average speed; it is
not a small additive ripple. These numbers describe this lightly damped
illustrative setup, **not typical motor performance**. Halving the outer
tick from 100 to 50 microseconds changes combined RMS error from 59.17%
to 59.15%; the ideal residual also shrinks. Automated tests cover the
ripple comparisons and convergence, torque identities/sign/periodicity,
current errors, detent with zero current, signed microstep commands,
loaded equilibrium, friction breakaway, explicit unit conversion, and
one-/two-body substep convergence.

Stepper parameters currently use the C API, not JSON configuration.
Non-sinusoidal phase torque and compensation experiments are next steps;
the ideal sine-law motor is not a complete microstep-accuracy model.

### Speed sweep and resonance

```
make examples
build/stepper_velocity --sweep > build/stepper_sweep.csv
python3 examples/plot_stepper_sweep.py
```

This runs the same six cases at 121 logarithmically spaced speeds from
.01 to 1.2 rad/s (.0955 to 11.46 rpm). It takes roughly two minutes on the
development machine. Each point is an **independent trial initialized at
ideal loaded traveling equilibrium**, not an acceleration ramp or a
pull-in/pull-out test. All use two seconds of settling, followed by at
least two seconds AND four complete commanded electrical cycles of
measurement. The window is rounded up to whole electrical cycles to
reduce endpoint bias. A shared fixture in `examples/stepper_experiment.h`
keeps the trace, sweep, and sweep regression tests on identical parameters.

The summary CSV includes target/mean speed, RMS error relative to target,
AC RMS (mean removed), peak-to-peak variation, backward-motion fraction,
and **unwrapped** maximum/final electrical load angle. RMS error and AC
RMS are absolute rad/s values; normalize by target for fractional ripple.
`phase_boundary_crossed` flags excursions beyond +/- pi load angle during
settling or measurement. This is a phase-well warning, not proof of
permanent loss of synchronism; the default sweep has none. A rotor can
reverse briefly within its magnetic potential well without missing a
step.

`build/stepper_sweep.{png,svg}` plots absolute and normalized RMS error,
mean-speed error, and backward-motion fraction. Dashed markers compare
the local small-signal resonance (~35.5 Hz) with excitation fundamentals:

| Excitation | Predicted crossing speed | Observed nearby result |
|---|---:|---|
| 64-microstep update rate | .0174 rad/s | ~.274 rad/s RMS error at .01748 rad/s |
| 16-microstep update rate | .0698 rad/s | ~1.06 rad/s RMS error at .0698 rad/s |
| Detent, 200 cycles/rev | 1.116 rad/s | ~.896 rad/s RMS error at 1.108 rad/s |

The prediction uses `sqrt(Kt*I*N*cos(loadAngle)/J)/(2*pi)` at the nominal
.2 rad/s loaded equilibrium, with damping neglected. Nonlinearity and
higher harmonics can shift peaks or create additional ones: the lower
speed peaks in the 16-microstep curve are not separate mechanical modes.
Large normalized error at low speed is not itself proof of resonance;
use the absolute-error panel too. In this lightly damped setup, some
runs move backward for nearly half the measurement window while still
tracking mean speed closely. These are model demonstrations, not motor
performance specifications.

Custom sweeps and timestep checks:

```
# MIN MAX POINTS [tick Hz]
build/stepper_velocity --sweep .01 .15 41 20000 > build/stepper_sweep_fine.csv
python3 examples/plot_stepper_sweep.py build/stepper_sweep_fine.csv build/stepper_sweep_fine
# All six cases at one speed, with explicit outer tick rate:
build/stepper_velocity --point .0698 20000
```

Regression tests check resonance against detuned points, the RMS/variance
identity, bounded load angle despite backward motion, original trace
metrics after refactoring, and peak convergence under halved timestep.
The strong 16/64-microstep and detent peaks change by less than .01% in
the targeted 10 kHz vs 20 kHz checks. **Sub-percent high-speed tails are
not equally converged**: pulse events occur only on outer ticks. For
example, at 1.108 rad/s the 64-microstep RMS error falls from .183% to
.068% when doubling tick rate. Do not interpret those small tails or
fine jagged features as calibrated physical effects. Resolving pulse
events inside a tick is a future numerical improvement; mechanical
substeps alone do not fix pulse-time quantization.

## Controller convention

There is exactly **one** public way to drive a simmy simulation with a
controller:

```c
typedef int32_t (*SimmyControllerFn)(void *userData, int32_t targetPosition,
                                      int32_t actualPosition, int32_t actualVelocity,
                                      uint32_t dtCycles);
```

The signature is chosen to match a typical fixed-point position-control
tick function closely enough that wrapping a real controller is a direct
passthrough, not an adapter fighting the API -- see
`examples/analog_servo_integration/main.c`, where the entire adapter for
a real production control cascade is:

```c
static int32_t ControlAdapter(void *userData, int32_t targetPosition, int32_t actualPosition,
                               int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  return ControlTick(targetPosition, actualPosition, actualVelocity, dtCycles);
}
```

The plain examples' own demo PD controller is just another ordinary
implementation of the same typedef -- no fallback/shortcut version.

## Bidirectional pulse source

`SimmyPulseTrainState`/`SimmyPulseTrainTickDelta` model the quantized
edge-count result a real step/dir (or quadrature) pulse train would
produce each tick for an idealized periodic source. The rate may be
**negative** -- a reversing move -- which is the only way to actually
exercise a coupling's backlash dead-zone (see `examples/bidirectional_move.c`):
a one-way move never crosses it. Jitter is clamped so it can never flip
the sign of a tick's increment -- it models short-term timing noise, not
a free direction reversal.

## JSON config files

Hand-rolled parser (`src/simmy_json.c`), scoped to exactly this shape --
not a general-purpose, hardened JSON parser.

```jsonc
{
  "bodies": [
    {"inertia": 64, "friction": 0.02, "stictionLimit": 0, "coulombLevel": 0,
     "gravityBias": 0, "rippleAmplitude": 60, "ripplePeriodCounts": 20,
     "frictionBumpCenter": 0, "frictionBumpHalfWidth": 0, "frictionBumpExtra": 0},
    {"inertia": 8, "friction": 0.01, "stictionLimit": 80, "coulombLevel": 50}
  ],
  "couplings": [
    {"springK": 3.16, "springD": 1.0, "backlashGap": 0.5}
  ],
  "feedbackBody": 0,
  "subSteps": 20,
  "analogChain": [
    {"type": "lag", "param0": 0.6}
  ]
}
```

`bodies` must be non-empty; `inertia` is required (no sane default --
there's no "how heavy is this body" default). Everything else defaults
to 0/disabled if omitted. `couplings` needs exactly `bodies.length - 1`
entries (extra ones are ignored, missing ones default to a no-op
coupling). `analogChain` stage `type` is one of
`gain`/`saturate`/`quantize`/`lag`/`noise`, using `param0`/`param1` per
stage (see `include/simmy.h`'s enum comments for what each means).

`SimmyChainParamsFromJsonFile` and `SimmyAnalogChainParamsFromJsonFile`
each read their own top-level section from the SAME file -- one JSON
file describes a complete scenario.

## Examples

- **`examples/basic_two_mass.c`** -- a JSON-driven two-body chain
  (`examples/configs/two_mass.json`), demo PD controller, CSV to stdout.
- **`examples/three_mass_leadscrew.c`** -- a three-body chain (motor ->
  belt -> lead screw -> nut [with backlash] -> table,
  `examples/configs/three_mass_leadscrew.json`). Also demonstrates
  reading a body other than the feedback one purely for measurement:
  prints the table's position alongside the motor's, so you can see the
  gap between "what motor-side control achieves" and "what actually
  happens at the load."
- **`examples/bidirectional_move.c`** -- a reversing move through the
  same three-body chain (its own config, `examples/configs/
  bidirectional_move.json`, with a larger nut backlash gap so the
  dead-zone crossing at reversal is clearly visible, not just 1-2
  ticks). Prints the nut's relative displacement and whether it's
  currently inside the backlash dead zone each tick, computed from the
  same public fields the library itself uses -- no special
  introspection API needed.
- **`examples/analog_servo_integration/`** -- optional, only builds if a
  `../analog-servo` sibling repo is present. Links the REAL
  `firmware/src/Control.c` from that repo and replays the exact scenario
  its own native test suite already calibrated, confirming the two
  reproduce matching numbers. This is the concrete proof that simmy
  composes with a real external control loop, not just its own demo
  controller.

## License

MIT, see `LICENSE`.
