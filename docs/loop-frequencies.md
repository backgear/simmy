# Loop frequencies

simmy has no opinion about control algorithms, but its examples fix
several clock rates so that results are comparable and reproducible.
This page collects them in one place.

## Mechanical chain

The chain's own tick has no fixed rate -- callers choose `dt` implicitly
through their unit conversions (see [the mechanical chain](mechanical-chain.md)).
Most examples use a "1 kHz-ish" outer tick. 2- and 3-body chains
sub-step the coupling dynamics internally (`SimmyChainParams.subSteps`,
default 20) because a realistic coupling stiffness at that tick rate is a
stiff ODE for plain Euler integration.

## Brushed DC

No clock of its own: ideal commutation removes the rotor-angle dependence
that gives every other motor here a sector, commutation, or microstep
rate to schedule. `SimmyDcStep` just runs on the mechanical tick, same as
`SimmyChainStep` itself -- there's nothing else to resolve.

## BLDC

| Clock | Rate | Notes |
|---|---:|---|
| Speed PI | 1 kHz | Ideal encoder feedback |
| Six-step commutation | 20 kHz | Sector selection from rotor angle |
| High-side PWM | 5 kHz | 40 timing quanta/period, 5 us digital quantum |
| Electrical/mechanical solver | 4 substeps/quantum by default (1.25 us) | `--steps` halves only solver dt |

The low-side leg of the selected sector stays ON during PWM off-time
(except at zero duty, where all gates are off). Because sector/PWM
decisions land on a fixed digital grid, refining the solver changes which
side of a quantization or sector boundary a trajectory crosses -- this is
a hybrid (continuous + clocked-logic) system, not a smooth ODE.

## Induction

| Clock | Rate | Notes |
|---|---:|---|
| Drive command (V/Hz, FOC, IFOC) | 1 kHz | Frequency ramp / speed-loop reference, fixed across all three drive modes |
| FOC / IFOC current loop | 10 kHz | Park-frame current PI + decoupling |
| Plant (electrical RK4 + mechanics) | 40 kHz (25 us/step) | Independent of the command/current clocks; `--steps` on each example refines only this grid |

The electrical and mechanical states share this plant integration grid;
the drive/current clocks are independent of it and stay fixed when an
example's `--steps` flag refines plant resolution. This is a partitioned
solver (RK4 electrical, semi-implicit mechanical), not a single coupled
high-order integrator.

## Stepper

| Clock | Rate | Notes |
|---|---:|---|
| Outer tick | 10-20 kHz (example-chosen) | Pulse/microstep commands update here |
| Mechanical substeps | `subSteps` under the outer tick, default 20 | Rotor position reevaluated every substep |

Resolving the magnetic/mechanical resonance **and** command pulse timing
both matter: mechanical substeps alone cannot recover a pulse edge hidden
inside an outer tick. The regression suite checks convergence between 10
kHz and 20 kHz outer ticks; sub-percent high-speed tails are not equally
converged at both, since pulse events only occur on outer ticks.

## Why these are all independent

Every example above deliberately keeps "what the controller/drive sees"
separate from "how finely the plant is integrated": a `--steps` (or
equivalent) flag on each example refines *only* the electrical/mechanical
solver grid, never the command, current, speed, commutation or PWM
clocks. That's what makes a convergence check ("does doubling the solver
rate change the answer?") meaningful without also changing what's being
modeled.
