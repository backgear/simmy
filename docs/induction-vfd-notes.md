# Averaged induction motor, open-loop V/Hz, and a path to FOC

## Scope

This is a balanced, unsaturated, squirrel-cage induction-machine model in
stationary alpha/beta coordinates, connected to simmy's mechanical chain.
It models induced rotor flux/current and electromagnetic torque, not
individual PWM switches. Example parameters are illustrative, not fitted
to a named motor. The example drive is open-loop V/Hz: it does not receive
rotor position, speed, current, or flux feedback.

The plant is deliberately separate from the drive. FOC replaces the
voltage-command generator, **not** the motor physics. FOC was not part of
the V/Hz milestone; a subsequent **[ideal true-flux FOC benchmark](induction-foc-notes.md)**
now implements the first controller stage on this same plant.

## Electrical equations and conventions

Amplitude-invariant Clarke coordinates, with alpha equal to phase A for
a balanced zero-sequence-free system:

```
a = alpha
b = -alpha/2 + sqrt(3)*beta/2
c = -alpha/2 - sqrt(3)*beta/2
```

Thus a balanced sine vector's magnitude is phase peak, not phase RMS or
line-to-line RMS. For balanced steady sinusoids only:
`Vphase_peak = sqrt(2/3)*Vline_line_rms` and
`Iphase_rms = hypot(Ialpha,Ibeta)/sqrt(2)`. The plotted instantaneous
current-vector envelope must not be described as transient phase RMS.

All rotor electrical parameters are referred to the stator. With
`Ls=Lls+Lm`, `Lr=Llr+Lm`, and `Delta=Ls*Lr-Lm^2`:

```
is = (Lr*psi_s - Lm*psi_r)/Delta
ir = (Ls*psi_r - Lm*psi_s)/Delta

d(psi_s)/dt = vs - Rs*is
d(psi_r_alpha)/dt = -Rr*ir_alpha - p*omega*psi_r_beta
d(psi_r_beta)/dt  = -Rr*ir_beta  + p*omega*psi_r_alpha

torque = (3/2)*p*(psi_s_alpha*is_beta - psi_s_beta*is_alpha)
magneticEnergy = (3/4)*(psi_s.dot(is) + psi_r.dot(ir))
```

`p` is pole pairs. Positive voltage-vector rotation produces positive
motoring torque below synchronous speed. The energy identity is:

```
dW/dt = (3/2)*vs.dot(is)
        - (3/2)*(Rs*|is|^2 + Rr*|ir|^2)
        - torque*omega
```

Electrical rotor speed is needed **inside the machine physics**; this is
not feedback to the open-loop drive. Unlike the stepper, rotor mechanical
angle is not itself a required magnetic orientation state for this
symmetric induction-machine model. Rotor flux has its own orientation.

Slip frequency is `fdrive - p*omega/(2*pi)`. Nonzero slip is normal under
load; a field/rotor speed difference is not automatically a fault or a
counted lost step. At ideal synchronous speed steady rotor current and
torque vanish (apart from discretization residuals). Above synchronous
speed, torque reverses and the machine can generate. The model allows
negative electrical input power, but there is no DC-bus energy model or
regeneration protection.

## Public pieces

- `SimmyInductionParamsValid`: finite/positive physical coefficient checks.
  Callers must validate before integrating, choose a sufficiently small
  timestep, and supply finite voltage/speed inputs. No adaptive solver.
- `SimmyInductionState`: four stator/rotor flux-linkage components.
- `SimmyInductionEvaluate`: actual stator/rotor currents, torque, stored
  magnetic energy. Currents are not prescribed by the drive.
- `SimmyInductionStep`: electrical RK4 with held voltage and mechanical
  speed; returns RK-weighted interval-average torque.
- `SimmyInductionChainStep`: body-0 speed enters the electrical model;
  average electrical torque drives body 0 via explicit SI/unit conversion.
- `SimmyChainStepDt`: advances existing mechanics by a fraction/multiple
  of its **original tick**, without changing its position/velocity units.
  Duration 1 preserves `SimmyChainStep` behavior bit-for-bit in tests.
- `SimmyVfdCommandStep`: sampled frequency ramp and V/Hz amplitude command.
- `SimmyVfdVoltageStep`: continuous phase advance at the held frequency,
  midpoint voltage for a plant integration interval. No command-clock tick.

The V/Hz law interpolates from a low-frequency boost intercept to base
phase-peak voltage at base frequency, then continues the slope until the
voltage ceiling. At exactly zero frequency it commands zero voltage: no
DC injection, energized position holding, or braking strategy. The
frequency ramp supports reversal. Zero commanded terminal voltage means
an ideal zero-voltage/shorted terminal condition, not a disconnected
open-circuit stator; flux can decay and induce current. Voltage is limited; current limiting,
trip logic, and speed regulation are not implemented.

## Independent command clock and integration resolution

This example has a fixed 1 kHz drive-command schedule and 40 plant steps
per command interval: **25 microseconds per plant step**. Changing to
`--steps 80` refines plant integration to 12.5 us while preserving exactly
the same command/ramp/load schedule and original chain units.

Electrical state uses RK4 with voltage and rotor speed held over each
plant step. Mechanics use a semi-implicit update with interval-average
torque. This is a **partitioned solver**, not a fourth-order fully coupled
machine/mechanics solver. The voltage oscillator's midpoint sample also
approximates the continuously rotating voltage within each plant step.
Do not infer fourth-order accuracy for the entire simulation.

The electrical and mechanical states currently share a plant integration
grid; the drive update grid is independent. A future multirate current
controller can run on its own fixed sample clock without changing either
motor equations or mechanical units. The older stepper examples have not
yet been migrated to this explicit-duration scheduling.

All runs are offline fixed-step calculations: no wall-clock deadlines,
sleep, real-time scheduling, or integration-step dropping. Scheduling
changes completion time only. Each run takes well under one second for
the numerical simulation on the development host; no sweep parallelism
is needed for these four examples.

Across all four traces, halving plant dt with the drive schedule fixed
changed sampled values by at most approximately:

| Quantity | Max absolute difference |
|---|---:|
| Rotor speed | .00384 rad/s |
| Electromagnetic torque | .00339 Nm |
| Stator-current vector magnitude | .00170 A |

These are checks for this parameter set and waveform, not universal
error bounds. Refine dt when changing electrical time constants, inertia,
or mechanical stiffness. Static friction retains the chain's existing
counts/tick near-zero threshold; unit mapping is deliberate here.

## Illustrative parameters

| Parameter | Value |
|---|---:|
| Pole pairs | 2 (four poles) |
| Stator resistance | 3 ohm |
| Referred rotor resistance | 2 ohm |
| Stator / rotor leakage inductance | .01 / .01 H |
| Magnetizing inductance | .18 H |
| Base frequency | 50 Hz |
| Base / maximum phase-peak voltage | 187.794 V (230 V line-to-line RMS) |
| Inertia | .02 kg m^2 |
| Viscous drag | .001 Nm/(rad/s) |
| Nominal command ramp | 25 Hz/s |
| Low-frequency comparison | 5 Hz, 10 Hz/s ramp, 0 or 10 V boost intercept |

The fixed original mechanical tick is .001 seconds and 1000 counts/radian,
with force unit Nm:

```
position_counts = 1000*theta
velocity_counts_per_tick = 1000*.001*omega
body.inertia = J/(1000*.001^2)
body.friction = B/(1000*.001)
```

Refining integration does not change these values. Brake load uses the
existing Coulomb/static-friction model. While moving it opposes rotation;
at standstill its specified capacity can hold less than that capacity.
It is **not a gravity torque** that would back-drive the rotor after stall.

## Experiments and figures

### Load, overload, and recovery

![Induction motor load and overload](plots/induction_vfd_load.png)

A 25 Hz/s command ramp reaches 50 Hz at two seconds. Apply a 6 Nm brake
at 3 s, increase to 20 Nm at 5 s, reduce to 6 Nm at 8 s. Field speed stays
1500 rpm while the motor slows or stalls; the open-loop drive does not
compensate load-induced speed error.

| State | Mean rotor speed | Slip frequency | Motor torque |
|---|---:|---:|---:|
| Unloaded, 2.8–3.0 s | 1498.43 rpm | .0522 Hz | .1569 Nm |
| 6 Nm brake, 4.8–5.0 s | 1429.52 rpm | 2.3493 Hz | 6.1497 Nm |
| 20 Nm brake, 7.8–8.0 s | 0 rpm | 50 Hz | 9.8763 Nm |
| Recovered, 10.8–11.0 s | 1429.52 rpm | 2.3493 Hz | 6.1497 Nm |

The overload passes the breakdown-torque region (roughly 16 Nm here)
and stalls the motor. The brake's **capacity** remains 20 Nm, but its
actual static reaction balances about 9.88 Nm of motor torque. Plotting
the capacity must not imply 20 Nm mechanical work/torque absorption at
standstill. No real VFD would necessarily tolerate this sustained current:
there is no protection or heating model here.

### Abrupt voltage/frequency application versus ramp

![Induction startup comparison](plots/induction_vfd_startup.png)

The direct 50 Hz start reaches speed quickly but has a sampled peak
stator-current vector of 27.09 A, versus 3.54 A for the unloaded ramp.
Both start with zero flux and mechanical velocity. This compares two
specific open-loop excitation profiles, not optimized drive tuning.
The plot includes rotor-flux buildup, which will matter for FOC startup.

### Low-speed voltage boost

![Low-speed induction motor voltage boost](plots/induction_vfd_low_speed.png)

At 5 Hz, apply a 3 Nm brake at 1 s. Without boost, this example stalls;
with a 10 V phase-peak boost intercept it sustains about 126.25 rpm
(field speed 150 rpm), with .7917 Hz slip. Boost restores flux/torque
margin lost to stator resistance, but does not regulate speed.

The boosted no-load startup reaches about .806 Wb turns rotor-flux
magnitude, above the base-frequency ~.565 Wb turns level. **The linear
magnetic model omits saturation**, so the setting should not be presented
as safe, efficient, or a recommended hardware setting. This flux plot is
included specifically to prevent “more boost is always better” conclusions.

## Validation

`make test` includes:

- Parameter validation; zero-voltage/zero-flux invariance even with a
  rotating rotor (no permanent-magnet torque hiding in the model).
- Flux/current inversion, stored-energy positivity, copper-loss decay,
  electrical power balance, and lossless mechanical-energy exchange.
- Independent steady-state per-phase equivalent-circuit agreement for
  locked rotor, motoring at .04 slip, and generating at -.04 slip.
- Direction symmetry and near-zero steady rotor current/torque at zero slip.
- Frequency ramp, voltage boost/clamp, zero-frequency voltage behavior,
  negative rotation, and unchanged ramp state during voltage integration.
- Backward compatibility of one-tick mechanical integration and explicit
  fractional-tick dynamics.
- Coupled load droop, passive-brake stall, recovery, low-speed boost
  comparison, and integration refinement with a fixed drive command clock.

This validates equations and implementation relationships, not agreement
with measured hardware. No PWM ripple, magnetic saturation, core loss,
skin/deep-bar effects, inverter dead time, DC-bus dynamics, mechanical
endstops, or winding temperature dependence are included. Parameters use
the C API, not JSON configuration, for this milestone.

## FOC roadmap: same plant, different controller

The induction model is voltage-in/current-and-flux-out, not a baked-in
V/Hz torque curve. The first two stages below are implemented in the
[ideal FOC experiment](induction-foc-notes.md), and the encoder-based
indirect variant in stage three now runs in the
[indirect FOC experiment](induction-ifoc-notes.md). A separate firmware
observer and sensorless control remain future work:

1. **Ideal rotor-flux-oriented control benchmark.** Take angle from
   `atan2(psi_r_beta, psi_r_alpha)` and magnitude from the simulated rotor
   flux. Park-transform the actual stator current. Use d-axis current to
   establish flux, q-axis current for torque, discrete current PI loops
   with voltage-vector saturation and anti-windup, then inverse Park to
   generate alpha/beta voltage for this same plant. Bootstrap flux first:
   its angle is undefined at zero magnitude.
2. **Separate current and speed sample clocks.** For example a 10 kHz
   current loop, 1 kHz speed loop, and finer plant integration. Keep all
   those rates fixed while refining only plant dt.
3. **Encoder-based indirect FOC / flux estimation.** Remove privileged
   access to true rotor flux. Integrate electrical rotor speed plus an
   estimated slip frequency, or use a rotor-flux observer; test parameter
   mismatch, especially rotor-resistance drift. Encoder angle alone is
   not rotor-flux angle, unlike a simple permanent-magnet model.
4. **Sensorless estimation later.** Use currents/voltages and a suitable
   observer; zero/low-speed observability is a separate problem. Do not
   call true-state flux feedback “sensorless FOC.”

In ideal rotor-flux coordinates, useful consistency equations are:

```
tau = (3/2)*p*(Lm/Lr)*|psi_r|*is_q
d|psi_r|/dt = (Lm*is_d - |psi_r|)/(Lr/Rr)
omega_slip = (Lm/(Lr/Rr))*is_q/|psi_r|
```

The q-axis sign must match the chosen Park transform; Clarke scaling
must retain the same 3/2 factor as the plant. Compare V/Hz and FOC on
speed droop, load rejection, current demand, flux control and voltage
headroom, with explicitly comparable actuator/protection limits. The
current V/Hz example has a voltage limit but no current protection.

## Reproduction and retained artifacts

```sh
make test examples
build/induction_vfd > build/induction_vfd.csv
python3 examples/plot_induction_vfd.py build/induction_vfd.csv docs/plots
# Refine only the plant solver; command and load schedule remain 1 kHz:
build/induction_vfd --steps 80 > build/induction_vfd_fine.csv
```

Numerical library/tests/examples need only C and libm; the independent
circuit regression also uses standard C complex arithmetic. Plotting
requires Python, NumPy and Matplotlib. The generator writes a compact
`build/induction_vfd_summary.csv`; a retained copy lives at
`docs/data/induction_vfd_summary.csv`. Full raw traces remain generated
under `build/`. PNG/SVG figures under `docs/plots/` survive `make clean`.

Possible future article angle: **“Slip is not a missed step.”** Start by
contrasting the stepper's discrete loss of synchronism with the induction
motor's normal load-dependent field/rotor speed difference. Then show
why V/Hz is not speed regulation, why flux matters at low speed, and how
FOC changes the excitation policy without changing the motor model.
