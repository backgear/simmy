# The mechanical body chain

A chain of 1 to `SIMMY_MAX_BODIES` (8) rigid bodies, each with its own
inertia, viscous friction, static friction (stiction + Coulomb, with an
optional Coulomb term that grows with |driving force|, `coulombPerForce`),
gravity bias, periodic ripple, and an optional one-off localized friction "speed
bump" (a dusty/less-greased section, distinct from the recurring
ripple). Adjacent bodies connect through a `SimmyCouplingParams`: spring
stiffness, damping, an optional backlash dead-zone, and an optional
transmitted-force ceiling -- the same struct whether that connection is
a flexible shaft, a leadscrew/nut, a belt, a gear mesh, or a friction
clutch; see [Couplings](couplings/index.md) for how to translate a real
transmission's ratio, stiffness, backlash, and slip torque into these
numbers.

A **1-body chain is exactly a rigid coupling** -- no spring math at all,
which is both physically correct for a truly rigid connection and
numerically the safest choice (no stiff-spring instability to worry
about). 2- and 3-body chains sub-step the coupling dynamics internally
(`SimmyChainParams.subSteps`, default 20) for numerical stability against
a stiff spring -- a realistic coupling stiffness at a 1kHz-ish tick is a
stiff ODE for plain Euler integration.

One body is the right model exactly when nothing between the motor and
whatever reads position back has inertia worth tracking on its own -- a
shaft with an encoder bolted straight to it, say. The moment something
in that path carries its own mass and can move even slightly
independently of the motor -- a flywheel, a real compliant coupling, a
belt-driven load -- that's a second body, joined through a
`SimmyCouplingParams`. Nothing stops you from chaining up to
`SIMMY_MAX_BODIES` (8), but real transmissions rarely reward it: most
mechanisms have one or two compliant modes worth resolving, and
`examples/three_mass_leadscrew.c` (motor -> belt -> leadscrew ->
nut/table) is already near where a model stops being worth the added
complexity. Add a body because the data shows you need it, not because
the chain allows it.

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

## Units and numerical resolution

Motor models use SI units; the chain itself works in counts/tick. Supply
explicit `radiansPerCount` and `forcePerNm` conversion factors. For
example, with `C` counts/radian, tick duration `dt` seconds, and chain
force unit Nm (`forcePerNm=1`):

```
position = C * theta            velocity = C * dt * omega
body.inertia = J / (C * dt^2)   body.friction = B / (C * dt)
radiansPerCount = 1 / C         body.gravityBias = load torque in Nm
```

Here `J` is kg m^2 and `B` is Nm/(rad/s). Couplings likewise need
conversion if starting from SI parameters. The chain's existing stiction
velocity threshold is in counts/tick, not rad/s; choose the scale
deliberately.

Parameters follow the library's caller-validates convention: positive
inertia, positive conversion factors, positive electrical cycle count.
No automatic timestep selection is provided -- check convergence when
changing tick rate or stiffness. See [loop frequencies](loop-frequencies.md)
for the rates the examples actually use.

## Bidirectional pulse source

`SimmyPulseTrainState`/`SimmyPulseTrainTickDelta` model the quantized
edge-count result a real step/dir (or quadrature) pulse train would
produce each tick for an idealized periodic source. The rate may be
**negative** -- a reversing move -- which is the only way to actually
exercise a coupling's backlash dead-zone (see `examples/bidirectional_move.c`):
a one-way move never crosses it. Jitter is clamped so it can never flip
the sign of a tick's increment -- it models short-term timing noise, not
a free direction reversal.
