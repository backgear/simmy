# Fit a chain to recorded motion

Describe the bodies, couplings and encoder locations, then fit a runnable simmy
configuration from effort and position recordings. The fitter calls the real
`SimmyChainStepDt` kernel through a batch C bridge; it does not command hardware.

## Build and run

Install NumPy and SciPy; plotting additionally needs Matplotlib. A C compiler
builds the bridge. From this directory:

```sh
./build.sh
python3 fit_chain.py demo --output /tmp/demo
python3 fit_chain.py fit /tmp/demo/demo_spec.json /tmp/demo/demo_data.csv --output /tmp/fit
python3 fit_chain.py validate /tmp/fit/fit.json HELD_OUT.csv SPEC.json
python3 -m unittest discover -s tests -p 'test_fit_chain.py'
```

The demo generates synthetic self-test data, not a measured machine. For your
own uniformly sampled CSV, supply `force` plus the configured encoder columns,
and pass the correct `--dt-ticks` to both fit and validate. The CSV time column
does not set the integration interval.

[Quick chain fit](../docs/characterization_fitter.md) describes the interface.
[Recordings and configuration](../docs/identification_recordings.md) covers units,
fixed/free parameters and independent validation.

## Guarantees and limits

- Output `fit.json` is a plain simmy chain configuration. `fit-report.json`
  records solver results and limitations separately.
- Smooth parameters use bounded least squares with an adaptive robust-loss
  scale. Free backlash gaps use a small coarse grid; one free gap additionally
  gets a bounded local refinement.
- Initial velocities and unobserved initial positions are nuisance estimates,
  not recovered physical parameters. Hidden bodies remain weakly identified.
- `--seed` compares a warm start with fresh guesses on the supplied fitting
  recording. Its `validation_rmse` report field is not a held-out test.
- More coefficients do not fix missing excitation or an incorrect topology.
  Freeze the output before evaluating another recording.

`build.sh` compiles only the batch bridge and the existing mechanical kernel
into the local ignored `build/` directory. It does not add Python dependencies
to the core library or require a sibling controller checkout.
