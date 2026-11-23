# Clutches

`SimmyCouplingParams.forceLimit` clamps a coupling's transmitted force
to a constant ceiling (see [Belts](belts.md#v-belts-and-poly-v-multi-rib-belts)
for where that came from). "Clutch" covers several mechanisms, and they
split into three genuinely different categories -- not just different
formulas for the same ceiling.

## Force ceiling (one mode) -- supported

**Plate/disc friction clutch.** A spring-loaded plate or disc clutch
(dry or wet) is the plainest case: a fixed normal force (from a spring,
set once at assembly) presses friction faces together, giving a
constant torque ceiling, `T_max = mu * F_normal * r_effective * n_faces`.
Below it, the clutch is effectively rigid (or has whatever compliance
the plates/springs add); above it, it slips continuously -- exactly the
same force-saturation shape as a V-belt past its capstan limit, just
with a different formula for where the ceiling sits. Convert that
`T_max` to the chain's internal force units and set it as `forceLimit`:
same mechanism, different ceiling source, already supported.

## Mode switch (bimodal) -- not supported

The next three aren't friction that merely *varies* -- each one has two
genuinely distinct operating regimes, switched by some other state
variable crossing a threshold. That's a different kind of gap than "the
ceiling isn't constant": it needs a mode (a bit of memory/hysteresis
beyond the instantaneous force law), not just a fancier formula for the
same law.

- **Centrifugal clutch** (go-kart and chainsaw clutches are the common
  examples): disengaged below a threshold speed, then centrifugal force
  on the shoes grows roughly with `omega^2` above it, ramping the normal
  force -- and so the ceiling -- from zero toward full lockup. The
  trigger is the clutch's own rotational speed.
- **Ball-detent (ball-ramp) torque limiter**: balls seated in
  spring-loaded pockets at specific angular positions; past a torque
  threshold they pop out and the clutch slips or ratchets, often
  re-seating once per revolution at the next detent rather than slipping
  smoothly and continuously. The trigger is accumulated torque/position,
  and the tripped regime is periodic, closer in spirit to a body's
  `rippleAmplitude`/`ripplePeriodCounts` detent machinery than to a
  smooth ceiling. Many of these are also one-shot overload protection
  rather than something meant to slip continuously in normal operation.
- **Compressor (A/C) clutch**: an electromagnetic plate clutch that's
  normally run fully engaged or fully disengaged, with no continuous
  partial-slip range in intended operation -- but the trigger is an
  *external command* (the solenoid is energized or not), not an
  internal state threshold. That makes it the simplest member of this
  bucket to eventually support: no hysteresis or threshold-sensing
  logic needed, just a commanded boolean mode switching between
  `forceLimit = 0` (freewheeling) and locked.
- **One-way/sprag/freewheel clutch** (a bicycle freewheel; also shows
  up *inside* a torque converter, below): rigid in one relative-velocity
  sign, free in the other. Same "two distinct regimes" shape as the
  others, just gated by the sign of slip rather than a magnitude.

## Continuous slip-curve (no ceiling at all) -- not supported

**Torque converter.** A fluid coupling with no hard ceiling at all:
torque transmits according to a continuous, nonlinear function of the
speed *ratio* between input and output -- multiplication near stall,
tapering toward roughly 1:1 as slip shrinks. That's a fundamentally
different shape of law than a clamp on `springK*rel + springD*relVel`;
it would need `torque = f(relative_velocity)` directly, not a limit
applied to the existing spring/damper terms. Real torque converters
often add a lockup clutch for efficiency once slip is small -- that
part is just the mode-switch bucket again, bolted on top -- and contain
their own internal one-way clutch on the stator, another mode-switch
example in the wild.

## Already free -- no new feature needed

Not every "coupling-sounding" mechanism is actually a gap:

- **Viscous/fluid coupling**: transmitted force proportional to
  relative velocity, no stiffness term at all -- that's just
  `springK = 0`, `springD` carrying the whole law. Already supported,
  today, no new field required.
- **Eddy-current/magnetic coupling**: its torque-vs-slip curve has the
  same shape as an induction motor's torque-vs-slip curve (same
  underlying physics -- one side just isn't externally powered).
  `SimmyInductionParams`' torque equation is a plausible direct reuse
  for this, redefining slip as the relative speed between the two
  coupling bodies instead of rotor-vs-stator-field slip -- a cross-link
  to an existing feature, not a new one.

**Summary:** a constant `forceLimit` covers a plain friction clutch and
a belt's slip limit. Centrifugal, ball-detent, compressor, and one-way
clutches all need an actual mode switch -- a state, not a formula -- and
none of them are supported today. A torque converter needs a different
law entirely (a continuous function of slip, not a ceiling on one).
Viscous and eddy-current couplings, on the other hand, aren't gaps at
all -- one's already free, the other reuses a feature this library
already has for a different reason.
