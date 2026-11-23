# Exercise a production controller against an empirical axis model

Link the sibling RLS/GainGlide/Control modules against frozen SM232AE/TQ10
command-v2. This is an optional simulation-only integration, not hardware tuning
or a replacement for the independently run PC tuner.

```sh
make rls-axis-command-v2
build/rls_axis_command_v2 --readme-demo /tmp/sim-frames.json 64000 4096 300 3276 16 1024 500 28 250
python3 -m unittest discover -s tests -p 'test_axis_readme_demo.py'
```

Run from the repository root. Set `ANALOG_SERVO_RLS_DIR` if the controller is not
in `../analog-servo-rls`. The sibling must supply the controller's integral-state
getter and velocity-reference interface. This executable never opens a serial
port. Its exploratory sweep can exceed the model's measured speed/effort range;
convergence is a simulation observation, not evidence of hardware agreement.

## Match the feedback interface

The README demo samples position once per 1 ms controller tick and feeds back
`position[n] - position[n-1]`. A ramp supplies both its accumulated position target
and its velocity reference. Omitting the latter creates outer-loop tracking lag;
rounding continuous model velocity is not equivalent to encoder differences.
Older exploratory RLS sweep paths still use that rounded-truth observation and
must not be described as an identical hardware measurement path.

Each frame records the state observed at time `t` and the command applied during
the following interval. The plant advances twice at 0.5 ms per controller call.
Its physical state remains continuous through the demo. The step window is
`2 * stepTicks`; the active inter-move gap is not a hardware disable/enable cycle.

Frames retain requested/applied velocity references, encoder delta, rounded truth
velocity, integral before/after, P contribution and unclamped candidate output.
For this demo's zero-filter/deadband/acceleration-feedforward configuration,
`output - stored_integral` matches P away from clipping, to integer rounding.
At saturation, anti-windup can reject an integral update, so the subtraction is
not generally P. These are effort codes, not measured current or torque.

`--readme-demo-no-reference` and `--readme-demo-legacy` are diagnostic controls
that deliberately restore missing-reference and old-feedback behavior. They are
not recommended defaults. The regressions keep the command clock and plant
parameters fixed while checking encoder consistency, reference routing and
integral units. No fitted timing alignment is used.
