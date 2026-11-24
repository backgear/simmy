# Test friction memory without driving the axis

These tools test whether stored friction load can explain recorded holding,
restart, and settling behavior. They are experimental. They do not replace the
frozen v2 plant, and the powered PC tuner must not silently accept their extra
parameters.

## Interfaces

- `axis_model_v3.py` fits recorded effort and encoder motion. It never runs a
  controller or opens a device. Use `--help` for input directories and bounds.
- `axis_v3_chain.py` replays complete acquisition suites continuously. It carries
  position, velocity, command-filter state, elastic load, and contact strength
  through the recorded idle gaps, including sub-sample clock remainders.
- `axis_v3_compare.py` compares independent capture-conditioned replays. It
  refuses to discard a state-carry candidate's initialization policy unless
  `--allow-conditioned-capture-reset` is explicitly requested for a diagnostic.
- `axis_v3_ramp_preview.py` runs the unchanged production controller independently
  against v2 and a candidate, with identical fixed gains and 1 ms encoder-delta
  feedback. It produces a **simulation-only** plot and a non-executable reference
  proposal, not a hardware acquisition plan.

The fitter holds v2's moving-loss, input-filter, and deadband coefficients fixed.
`--refine-gains` additionally permits the two command gains to move within eight
percent of v2. The default objective normalizes by recorded amplitude; the
`tolerance` alternative records its absolute floors and relative scales in the
registration. Costs from different normalizations are not directly comparable.

## State and provenance

A complete suite starts with assumed rest, zero elastic load, and fully healed
contact strength. These initial conditions are not fitted and are not measured.
Within the suite, no maneuver or capture resets the physical state. Separate
suites and older standalone captures remain independent when the intervening
input history is unavailable. Never invent a zero-input bridge across unknown
powered activity. Independent-capture rest assumptions require the explicit
`--independent-capture-rest-assumption` fitter override.

Replays use recorded command effort, not references or controller gains. Models
and raw inputs are hashed; fit runs retain their kernel sources. A quiet encoder
does not establish zero internal load. Neither commands nor fitted acceleration
coefficients establish current, torque, stage gain, or a particular mechanical
friction element. The legacy cross-firmware ramp has unqualified scale transfer
and is excluded from fitting.

## What must pass before replacement

Check old moving/coast data, weak/restart suites, and controller workloads—not
just the workload used to choose a candidate. Inspect cumulative drift with
state carry as well as local velocity and settling. The native adapter must
match the original batch kernels and preserve state across arbitrary splits;
unit tests also cover zero-command passivity and timestep convergence.

Similar independently controlled positions do not prove same-input plant
accuracy. Seen development recordings are not independent physical validation.
A new real ramp requires fresh powered-test authorization and health/travel
checks. Zero effort does not brake or isolate the jumper-enabled drive.
