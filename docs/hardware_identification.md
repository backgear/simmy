# Fit a model, then test what you need it to do

Use recorded effort and encoder motion to bring a model closer to your machine.
Start with a quick fit, run your control software against it, and refine only the
behavior that matters for your application.

## Fit your machine

- [Quick chain fit](characterization_fitter.md): describe bodies, couplings and
  encoder locations; get a runnable configuration from your recordings.
- [Recordings and fit configuration](identification_recordings.md): CSV fields,
  units, fixed/free parameters and training/validation separation.

The generic fitter steps simmy's mechanical chain directly. The measured axis
example below uses a separately identified empirical command model; it is not
presented as an output of that generic fitter.

## Measured example: SM232AE / TQ10

Two different questions need two different tests:

| Question | Test | What to inspect |
|---|---|---|
| Can I exercise the same control software in simulation? | [Controller development](axis_controller_development.md) | Independently tuned position responses and settling |
| Does the plant predict motion for a known input? | [Predicting recorded motion](axis_recorded_motion.md) | Effort replay and position residuals, without feedback correction |

A model may be useful for controller development while still needing work on
holding forces, low-speed motion or long coasts. Keep those conclusions separate.
Refinement should cover both operating ranges, not merely lower training error.

## Reference

- [Recorder](axis_recorder.md) and [device protocol](axis_device_protocol.md).
- [Acquisition and identification walkthrough](axis_identification.md).
- [Reproduction](identification_reproduction.md): source/config identities,
  archive hashes and offline commands. Bulk recordings stay outside Git.
