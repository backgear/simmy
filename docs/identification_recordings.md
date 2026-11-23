# Recordings and fit configuration

Record the effort sent to the mechanism and the positions that result. The
[quick chain fitter](characterization_fitter.md) reads those files; it does not
command the machine or design its ongoing workload.

## CSV input and timing

Use a `force` column and one position column per configured encoder:

```csv
time,force,motor_position,table_position
0.000,0.0,0.000,0.000
0.001,0.2,0.001,0.000
0.002,0.2,0.003,0.001
```

These rows illustrate the format, not measured behavior. The fitter uses row
order and the explicit `--dt-ticks` sampling interval; it does **not** infer or
resample the interval from `time`. Supply a uniformly sampled file with consistent
time, position, inertia and effort units. For a seconds-based configuration
sampled at 1 kHz, pass `--dt-ticks 0.001` to both fit and validate.

`force` must be actual recorded applied effort, not a requested position or
velocity. If it is a normalized command rather than calibrated force/torque,
the fitted coefficients are effective command-unit coefficients, not measured
physical inertia or friction. Record scale, polarity, clipping and output timing.
Do not repair a sign or time mismatch with fitted plant parameters.

## Describe the mechanism

A fit-spec mirrors simmy's [JSON configuration](build.md#json-config-files).
A plain numeric leaf is fixed; a `guess`/`min`/`max` object is free:

```json
{
  "bodies": [
    {"inertia": {"guess": 50, "min": 1, "max": 500}, "friction": 0.02},
    {"inertia": 8, "friction": {"guess": 0.05, "min": 0, "max": 1}}
  ],
  "couplings": [
    {"springK": {"guess": 2, "min": 0.1, "max": 50}, "springD": 2, "backlashGap": 0}
  ],
  "encoders": [
    {"column": "motor_position", "body": 0},
    {"column": "table_position", "body": 1}
  ],
  "feedbackBody": 0,
  "subSteps": 20
}
```

`encoders` is the fitter-specific addition: it maps CSV columns to bodies. Effort
drives body 0, matching `SimmyChainStep`. Choose the topology from the machine;
more free parameters cannot create information absent from the recordings.
A motor encoder alone cannot uniquely identify hidden compliance or backlash.

## Keep fitting and validation separate

Preserve raw files, timestamps, settings and faults. Use one set to fit and a
separate set to evaluate the frozen output:

```sh
python3 fit_chain.py fit SPEC.json TRAIN.csv --dt-ticks 0.001 --output /tmp/fit
python3 fit_chain.py validate /tmp/fit/fit.json HELD_OUT.csv SPEC.json --dt-ticks 0.001
```

The warm-start comparison made by `fit --seed` evaluates alternatives on the
provided fitting recording. Its report field named `validation_rmse` is not an
independent held-out test. Once you use a validation result to choose or improve
a model, label that recording seen development evidence.

The fitter estimates initial velocities and hidden-body initial positions during
training. Keep those nuisance estimates distinct from physical parameters and
from the initial conditions available when predicting a new capture. Do not
reset physical state at each maneuver to manufacture an accurate replay.

See [the recorder](axis_recorder.md) for the encoder rig's raw-file format and
[reproduction](identification_reproduction.md) for existing measurement evidence.
