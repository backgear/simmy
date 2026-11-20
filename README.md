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
  with period/amplitude picking which real effect it represents.
- The stiction model is the textbook "decide before integrating whether
  the driving force can overcome static friction" formulation -- it
  avoids the classic bug where a naive post-hoc check makes velocity
  chatter around zero, but it's not a claim about any particular real
  friction surface.
- Not yet modeled (future work): **endstops/hard limits** -- position
  thresholds that produce a contact event (not just a spring, a
  one-sided hard stop), for studying home-overshoot at different
  approach velocities and deceleration/smoothing profiles.

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
