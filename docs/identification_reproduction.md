# Reproduce a controller comparison offline

Replot saved feedback and rerun simulation without connecting a motor. Keep the
exact sources and configuration with a result; a model name alone does not
identify the code or input data that produced it.

## Save enough to rerun

`examples/axis_pc_tune.py` records the registered search policy, source hashes,
frozen model, trial history and validation result in its output directory.
Hardware outputs additionally contain raw streams and integrity reports. The
shared controller's `source_id.json` identifies its source files, not an
authenticated firmware binary. Retain full firmware sources and ELF/BIN hashes
when binary identity matters.

Bulk recordings and run reports belong outside Git. `examples/axis_pack_dataset.py`
creates deterministic compressed archives with per-file inventories and hashes;
use its `--help` for input/output options. Preserve failed captures as diagnostic
evidence, not successful identification inputs.

The measured SM232AE/TQ10 example's compact metrics and archive index are under
`datasets/axis-sm232ae-tq10/pc-servo-expanded-01/`. `artifacts.json` lists archive
sizes and SHA-256 hashes. **Public download hosting is pending**; an archive
filename or local storage path is not a public download URL.

## Read saved results

With the two complete tuner outputs available, install NumPy and Matplotlib:

```sh
python3 examples/axis_pc_compare.py --sim SIM_RUN --hardware HARDWARE_RUN --output /tmp/comparison
python3 examples/axis_docs_figures.py --sim SIM_RUN --hardware HARDWARE_RUN --output /tmp/figures
```

Both commands read files; neither opens a serial port. The first exports full
position/effort diagnostics and metrics. The second gives a compact position and
settling comparison. Add `--replay CASE.npz` to the second command for a separate
recorded-input prediction/residual figure.

Do not align curves by fitting a time shift or reset physical state at maneuver
boundaries. Preserve the command clock, initial-state convention and model order.
The plotting tools validate the captured references and available integrity
metadata before rendering.

## Rerun a frozen simulated search

With a C compiler and NumPy, use the source/config snapshot from the result:

```sh
python3 SOURCE/examples/axis_pc_tune.py --backend sim --model MODEL.json \
  --policy expanded-v1 --finalist-repeats 3 --output /tmp/reproduced-sim
```

Compare full candidate histories, gains, scores and validation metrics, not just
the final image. The measured example's 53-trial simulated history reproduced
identically from its retained snapshot. Changes to current working sources are
not a rerun of that snapshot.

## Powered acquisition is a separate operation

The recorder, tuner and adapter CLI honor the local
`build/axis-validation/POWERED_TESTS_PAUSED.json` latch. They do not clear an
external drive fault automatically. This host guard is not a physical interlock,
brake or removal of a drive's jumper enable.

Before a newly authorized effort replay, verify sign, scale, interval timing,
baseline state and travel guards. A waveform plotted as an integral contribution
or an integral-adjusted diagnostic is not automatically a valid actuator command.
