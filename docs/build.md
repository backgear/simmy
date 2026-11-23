# Build

```
make lib        # build/libsimmy.a
make test       # plain assert()-based test suite
make examples   # self-contained mechanical and motor experiments
make integration      # OPTIONAL -- only if a ../analog-servo sibling repo exists
make rls-integration  # OPTIONAL -- only if a ../analog-servo-rls sibling worktree exists
```

Only dependencies: a C compiler and libm. No Python, no third-party JSON
library -- the config parser is hand-rolled (see "JSON config files"
below). Plotting examples' output (optional, see [examples](examples.md))
additionally needs Python, NumPy and Matplotlib.

`make integration` links the REAL `firmware/src/Control.c` from a sibling
`analog-servo` repo and replays a scenario its own native test suite
already calibrated. `make rls-integration` additionally links
`RLS.c`/`GainGlide.c` from a sibling `analog-servo-rls` worktree to
exercise its online, in-move velocity-loop self-tuner. Both are skipped
(not failed) when the sibling isn't present.

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
    {"springK": 3.16, "springD": 1.0, "backlashGap": 0.5, "forceLimit": 0}
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

### Motor sections

Each motor's physical parameters load the same way, through their own
top-level key in the same file -- so one JSON file can carry the
mechanics above and a motor section together. Everything not shown
defaults to 0/disabled, just like the mechanical sections; fields with
no sane default (the ones each motor's own physics requires to be
positive) are called out on each [motor page](motors/dc.md). In rough
order of complexity:

```jsonc
{
  "dcMotor": {
    "resistance": 0.6, "inductance": 0.002, "backEmfConstant": 0.08
  }
}
```

`SimmyDcParamsFromJsonFile` reads `dcMotor`, then runs the result through
`SimmyDcParamsValid` -- `inductance` is required (must be positive).

```jsonc
{
  "bldcMotor": {
    "polePairs": 4, "connection": "wye", "waveform": "trapezoid",
    "resistance": [0.6, 0.6, 0.6], "inductance": [0.002, 0.002, 0.002],
    "backEmfConstant": [0.08, 0.08, 0.08]
  },
  "bldcBridge": {
    "busVoltage": 24, "diodeDrop": 0.6,
    "onResistance": [0.04, 0.04, 0.04, 0.04, 0.04, 0.04]
  }
}
```

`SimmyBldcParamsFromJsonFile` reads `bldcMotor` (`connection` is
`"wye"`/`"delta"`, `waveform` is `"sine"`/`"trapezoid"`) and
`SimmyBridgeParamsFromJsonFile` reads `bldcBridge`, each validated the
same way. `inductance` is required per winding; `busVoltage` is required.
Fault-injection fields (`openWinding`, `openSwitch`) aren't part of this
schema -- they're how the fault examples perturb an otherwise healthy
motor at runtime, not a sample-motor parameter, so they stay at their
zero/disabled default.

```jsonc
{
  "pmMotor": {
    "polePairs": 4, "waveform": "trapezoid", "resistance": 0.6,
    "inductance": 0.002, "backEmfConstant": 0.08
  }
}
```

`SimmyPmParamsFromJsonFile` reads `pmMotor` for the reduced two-current
PM model (`SimmyPmStep`), as opposed to `bldcMotor`'s explicit
winding/inverter model.

```jsonc
{
  "inductionMotor": {
    "polePairs": 2, "statorResistance": 3, "rotorResistance": 2,
    "statorLeakageInductance": 0.01, "rotorLeakageInductance": 0.01,
    "magnetizingInductance": 0.18
  }
}
```

`SimmyInductionParamsFromJsonFile` reads `inductionMotor`, then runs the
result through `SimmyInductionParamsValid` -- both leakage inductances
and the magnetizing inductance are required (must be positive).

```jsonc
{
  "stepperMotor": {
    "electricalCyclesPerRev": 50, "torqueConstant": 0.4,
    "detentAmplitude": 0.012, "detentCyclesPerRev": 200
  },
  "stepperCurrentPath": {
    "gainError": [0.03, -0.03], "offset": [0.01, -0.015]
  }
}
```

`SimmyStepperParamsFromJsonFile` reads `stepperMotor`;
`SimmyStepperCurrentParamsFromJsonFile` reads `stepperCurrentPath`.
`electricalCyclesPerRev` is required; `detentCyclesPerRev` is required
only once `detentAmplitude` is nonzero.
