# Preset: SM232AE / TQ10, free shaft (`sm232ae_tq10_noload`)

!!! warning "Fitted to one bench capture, not independently validated"
    Every number here comes from one closed-loop capture on one day. No
    held-out recording has been checked against it.

`models/sm232ae_tq10_noload.json` is a ready-to-run plant for testing
the analog-servo firmware against a Parker/Compumotor SM232AE motor on a
TQ10 drive, with nothing on the shaft. It answers one question: run the
real `Control.c` cascade against it, and do the step and ramp responses
look like the bench? For the readme move sequence they do: the torque pulse
widths, the velocity peaks and the rest chatter all line up (table below).

## Using it

The preset is an ordinary simmy config: one rigid body plus an analog
chain that stands in for the PWM RC filter, the TQ10's current loop and
its noise. One thing is not ordinary. The analog-chain parameters are per
**call**, and the fit called the chain 10 times per 1 ms tick
(`analogStepsPerTick` in the file). Run it the same way, or the delay,
slew and noise mean something else:

```c
SimmyChainParams chain; SimmyAnalogChainParams drive;
SimmyChainParamsFromJsonFile("models/sm232ae_tq10_noload.json", &chain, err, sizeof err);
SimmyAnalogChainParamsFromJsonFile("models/sm232ae_tq10_noload.json", &drive, err, sizeof err);
SimmyChainState shaft; SimmyAnalogChainState amp;
SimmyChainInit(&shaft, &chain);
SimmyAnalogChainInit(&amp, seed);              /* noise seed is the caller's */

for (;;) {                                      /* one 1 ms tick */
  int32_t position = SimmyChainFeedbackPositionCounts(&shaft, &chain);
  int32_t torque = MyControlTick(position);     /* firmware torque units, axis frame */
  for (int i = 0; i < 10; i++) {                /* analogStepsPerTick */
    double t = SimmyAnalogChainStep(&amp, &drive, torque);
    SimmyChainStepDt(&shaft, &chain, t, 0.1);
  }
}
```

Units:

- Position: encoder counts, 4096 per rev (quadrature edges).
  `SimmyChainFeedbackPositionCounts` rounds to whole counts, which acts as
  the encoder's quantization. That matters: the ±1-count dither at rest
  comes from it.
- Time: ticks of 1 ms.
- Force: analog-servo firmware torque units, full scale ±32767, in the
  axis frame (the firmware multiplies by the learned output polarity
  *after* this; telemetry `torque` is that physical value). These are
  command units, not N·m or amps.

## The model

| Piece | Value | simmy feature |
|---|---|---|
| Inertia | 4440 torque units per count/tick² | `bodies[0].inertia` |
| Transport delay | 0.062 tick | `delay` stage, 0.62 calls |
| Slew limit | 71,400 torque units/tick | `slew` stage, 7139 per call |
| One-pole lag | τ = 0.077 tick (nearly negligible) | `lag` stage, α = 0.729 per call |
| Coulomb friction | 121 + 5.4% of \|shaft torque\| | `coulombLevel`, `coulombPerForce` |
| Viscous friction | 4.63 per count/tick | `friction` |
| Stiction | 1135 | `stictionLimit` |
| Torque noise | uniform ±2253, every call (10 kHz) | `noise` stage |

Three of these pieces fill specific gaps:

- **Slew, not just lag.** From rest, a full-scale command reaches the
  shaft in about 0.3 tick. A full reversal (−30000 to +30000) takes about a
  full tick. A linear lag or delay can't do both, since it takes the same
  time for any swing. A slew limit does: a 60,000 swing takes twice as long
  as a 30,000 one. The best fit found without it scored 138 against 1.3
  with it, on the same step/ramp cost.
- **Torque-proportional Coulomb.** At the ±30000 clamp the bench brakes
  faster (7.3–7.8 counts/tick²) than it accelerates (6.4–6.6). That implies
  about 1500 of drag at full torque. On the slow ramp, at the same speeds
  and low torque, the drag is only about 450. A friction level that grows
  with |torque| fits both. It is a stand-in. A drive that delivers less
  than commanded while motoring and more while braking would look the same.
  The physical cause is not identified.
- **Noise.** Without it, the simulated shaft sits still at rest. On the
  bench it dithers ±1 count about 114 times a second, with ±1000-class
  torque kicks. The noise is applied per call. If you change the call rate,
  the effective noise changes too, and it would need refitting.

## How it was fitted

- **Source:** `/tmp/claude-1001/sv/readmefinal.json`, captured
  2026-11-26 on the F103 (72 MHz, 1 kHz tick) with analog-servo
  `tools/capture_moves.py`. That run was the readme sequence:
  1. `test square position 1024 400 400`, an unramped 1024-count step out
     and back.
  2. A velocity triangle: peak 60 counts/tick at a trajectory accel limit
     of 50.
- **Controller:** the saved tune. `velocity.kp` 385024, `velocity.ki` 3500,
  `position.kp` 32, filter shift 1, accel feedforward 467000, torque
  ±30000.
- **Method:** closed loop. The real analog-servo `Control.c`, `Signal.c`
  and `MotionQueue.c` (branch `motion-planner-experiments`) drove the
  candidate plant through the same sequence, tick for tick. Each candidate
  was scored on:
  - the step-out/back position, velocity and torque traces;
  - the ramp's per-phase mean torque and mean following error;
  - the 150 ms post-step creep;
  - the rest torque spread and dither rate.

  Search was differential evolution, then Nelder-Mead, with noise seed 1.

## What it reproduces

Sim columns use the same sequence and controller. "Seeds 1–5" is the
range over noise seeds.

| Metric | Bench | Sim (seed 1) | Sim seeds 1–5 |
|---|---:|---:|---:|
| Step out: settle to ±5 counts (ms) | 38 | 50 | 45–50 |
| Step out: overshoot (counts) | 0 | 1 | 1 |
| Step out: ticks at ±30000 clamp | 20 | 21 | 21 |
| Step back: settle to ±5 counts (ms) | 96 | 51 | 47–51 |
| Step back: overshoot (counts) | 0 | 1 | 1 |
| Step back: ticks at ±30000 clamp | 20 | 20 | 20–21 |
| Ramp: max \|following error\| (counts) | 5 | 3 | 2–3 |
| Ramp: RMS following error (counts) | 1.16 | 0.87 | 0.86–0.89 |
| Rest: torque SD | 339 | 322 | 294–346 |
| Rest: torque p1–p99 span | 1904 | 1882 | 1853–1892 |
| Rest: ticks with nonzero velocity, per s | 114 | 118 | 100–138 |

## What it gets wrong

- **Creep after the step back.** The bench sits 5–8 counts short of
  target for about 100 ms before creeping in; the sim settles in about
  50 ms. The noise that reproduces the rest dither also breaks stiction
  during that creep.
- **Settling after the step out** takes the sim about 10 ms longer to get
  inside ±5 counts.
- **Ramp breakaway.** The bench's 4–5 count following-error spike at the
  start of the ramp is only 1–2 counts in the sim.
- **Overshoot.** The sim overshoots 1 count where the bench shows none.
- **No current-loop ringing.** After a torque reversal the bench briefly
  overshoots its acceleration (about 9 counts/tick² against a steady 7.5);
  the slew and lag here cannot ring.
- **Overfit to one noise realization.** On other seeds the fit's cost
  is 6–7.5 instead of 3.8. The metrics above stay stable.
- **Narrow operating range.** One free shaft, one tune, one day, one
  sequence; speeds up to about 80 counts/tick.

## Relation to the frozen command-v2 axis model

This preset does **not** supersede
`models/axis/sm232ae_tq10_command_v2.json` (see
[Controller development in simulation](axis_controller_development.md)
and [Predicting recorded motion](axis_recorded_motion.md)). They are
different models of the same motor and drive, built different ways:

| | command-v2 | `sm232ae_tq10_noload` |
|---|---|---|
| Input | effort code / 32767, uploaded through the characterization rig | analog-servo F103 torque command (RC-filtered PWM), ±32767 |
| Units | rad, s; integrated at 0.5 ms | counts, 1 ms ticks, firmware torque units |
| Fitted from | 100 open-loop pulse/coast episodes, effort replay | one closed-loop capture, the real controller in the loop |
| Form | directional gains, constant + viscous loss, command filter, deadband | one body; delay, slew, lag, noise; Coulomb + torque-proportional Coulomb, viscous, stiction |
| Code | its own integrator (`examples/axis_command_model_v2.c`) | plain simmy chain + analog chain |

As a rough cross-check only: converted to command-v2's units, this
preset's full-scale acceleration (32767 / 4440 counts/tick²) is about
11,300 rad/s² per unit command. Command-v2's directional gains are about
13,000. The two are not reconciled here. They came through different
output stages (the rig's command path vs. the F103's RC-filtered PWM),
and neither measured current. Do not treat that gap as a measured gain
error in either model.
