# Scope and non-goals

Every model in simmy is a deliberate simplification, picked to answer a
specific question cheaply rather than to reproduce a specific machine.
This page says where each boundary sits and why, so you can tell whether
a result here answers your question or just looks like it does. Example
configs are representative ratios for recognizable mechanisms, not
measurements of a named machine -- do not cite a number from this repo as
a spec for real hardware.

## Modeling is leverage; perfect tuning is work

Simulating a plant and tuning a controller for one specific product
look like the same problem from a distance -- both need an honest
model of friction, backlash, inertia, and the electrical dynamics
underneath, and a lot of what you'd learn chasing one helps with the
other. But the constraints and the payoff are different enough that
this project treats them as different jobs, and only takes on the
first one.

The constraints differ first. simmy only has to be correct and fast on
whatever builds and runs it -- doubles, no real-time deadline, as much
history and compute as a laptop or a CI box has to spare. The actual
control loop for one product has to be correct inside an interrupt
budget on a resource-constrained microcontroller, often in fixed
point, with exactly the sensors that shipped and none of the ones that
didn't. Chasing that second kind of precision here would mean modeling
one firmware's limits instead of the underlying physics -- which
defeats the point, since a model worth keeping has to outlive any one
target, board, or product revision.

The payoff differs more. A better plant model is reusable: every
future motor, mechanism, or "will this survive backlash" question gets
to reuse it for free, and poking at it rewards curiosity -- change a
parameter, rerun in milliseconds, see the new shape. Finding the exact
autotune that gets one product's settling time down another 10% is
real, valuable work, but it's bounded and specific -- it pays off once,
for that product, and mostly rewards grinding through that product's
own quirks rather than anything that transfers. simmy is built to make
the first kind of work cheap, and to make the second kind easier to
check before committing hardware time to it -- not to replace it.

## Control is the caller's problem

simmy takes a torque or voltage and returns a new state; it has no
opinion about how that torque or voltage was chosen. A servo example
passes it through `SimmyControllerFn`, a plain function pointer (see
[driving simmy with your own controller](integration.md)). A three-line
demo P loop and a production control cascade linked in from another repo
both satisfy that same typedef the same way -- there is no separate
"real" calling convention hiding behind the demo one.

Closing the loop is likewise fixed to **one body**,
`SimmyChainParams.feedbackBody`, the one a real motor-mounted encoder
would read. You can still read any other body's position or velocity --
a table at the far end of a lead screw, say -- but only to measure
against, never to close the loop on. That's what lets an example show
the gap between what motor-side control achieves and what actually
happens at the load; closing the loop against a non-feedback body would
erase the thing the comparison exists to show.

## Where disturbance lives

A mechanical body's ripple field (`rippleAmplitude`/`ripplePeriodCounts`)
can stand in for cogging or commutation disturbance at the level of a
single sinusoid added to torque. The stepper and BLDC models instead
compute torque from actual phase currents and rotor angle, which is what
you need for a microstepping or commutation question. Using both at once
on the same mechanism double-counts the same electromagnetic effect under
two different names, so pick one. A separately observed mechanical
cogging effect can still ride on the ripple field even when a dedicated
electrical model is in use elsewhere in the chain.

## What a few specific models assume

**Stiction** decides, before integrating, whether the driving force can
overcome static friction. Decide first and you avoid the classic bug
where a post-hoc check makes velocity chatter around zero as the solver
flips back and forth across the threshold; this says nothing about any
particular real friction surface.

**Off-axis spring effects** -- a preload spring, or a misalignment that
would turn friction into a function of position through a changing
normal force -- are out of scope on purpose. `frictionBump*` is a flat
localized patch along the travel, not a continuously varying spring
coupling, and the two are not meant to approximate each other.

**Endstops and hard limits** don't exist yet: nothing produces a contact
event at a position threshold, so there is no way today to study
home-overshoot at different approach velocities or deceleration
profiles. This is the one item on this page that's a gap to fill, not a
boundary to keep.

**A coupling's torque ceiling** is a constant, via `SimmyCouplingParams.forceLimit`
-- it clamps `springK*rel + springD*relVel` to `+-forceLimit` (0, the
default, leaves a coupling unlimited). That covers a friction drive (a
V-belt or poly-V belt, as opposed to a positive-engagement timing belt
or gear) or a plain friction clutch once load exceeds what the belt's
tension/friction, or the clutch's spring preload, can hold (see
[Belts](couplings/belts.md) and [Clutches](couplings/clutches.md)). What
it doesn't do: track the slipped-away energy anywhere (the discarded
force just vanishes, with no heat/wear ledger), or add an actual mode
switch -- centrifugal, ball-detent, compressor, and one-way clutches
each have two genuinely distinct operating regimes gated by some other
threshold (speed, torque, an external command, or slip direction), which
needs a state, not a fancier ceiling formula. A torque converter is a
third gap again: no ceiling at all, just a continuous function of slip.
All of this remains a gap, not a boundary.

**The brushed DC, BLDC, and induction models** have no PWM switching, no
saturation, no core losses, no current trip, no DC-bus dynamics, and no
mutual inductance or shorted-turn circuit. The brushed DC and BLDC/PM
models do track electrical heat (resistive, switching, diode losses, in
joules) -- but that's energy dissipated, not a temperature: there's no
thermal mass, no cooling, no temperature state at all, and so no
winding-temperature feedback into resistance either. They answer "does
this control structure behave correctly on this electrical plant" and,
for brushed DC/BLDC, "how much energy does a fault cost," not "how hot
does this winding get" or "what happens during a shoot-through fault."
The brushed DC model goes one step further: ideal
commutation means it has no rotor-angle dependence at all, so it also
leaves out armature reaction and brush/commutator switching transients --
real brushes only approximate that idealization.

**The stepper model** is low-speed and current-driven. It has no winding
inductance, no chopper, no voltage limit -- give it a current and it
returns torque. That means it cannot predict the high-speed torque
envelope, where those effects start to matter.

## Determinism

No part of the physics -- body integration, any of the four motor
models -- calls `rand()`, reads the clock, or touches any other source
of entropy. The one place simmy has pseudo-randomness at all is two
named, optional features: the analog chain's `NOISE` stage and
pulse-train jitter, both built on a tiny internal xorshift32 PRNG
(`src/simmy.c`). That PRNG is itself deterministic -- caller-seeded,
and even the unseeded path falls back to a fixed constant, never the
clock -- so "deterministic" means what it should: the same config and
the same seed give the same answer, bit for bit, on any machine, not
"no randomness modeled anywhere."

## Why the test suite is still plain asserts

A test's job here is to keep working silently -- to prevent a future
regression, not to fail today. Plain `assert()` already does that: it
stops on the first wrong value, at the exact line, with the exact
expression that was false. Nothing about richer assertion matchers or
fancier failure reporting changes whether that regression gets caught,
so there's nothing to buy with a framework like Google Test that `assert()`
doesn't already cover for what this suite actually needs.

What a framework like that would cost is real, though: a C++ toolchain
and a Google Test dependency, on top of the project's "a C compiler and
libm" promise. We don't need it, and the simpler dependency fits the
project's goals better. The suite stays plain `assert()` until some
test need actually justifies that cost.
