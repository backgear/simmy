# The analog signal chain

A composable "train of analog type values" standing in for a real
DAC -> amplifier -> motor signal path: an ordered list of simple stages
(`gain`, `saturate`, `quantize` -- a DAC's LSB step, `lag` -- one-pole
electrical bandwidth, `noise`, `delay` -- fractional transport delay,
`slew` -- a rate limit), each transforming a single scalar value.
Nothing requires using all of these or in any particular order --
compose whatever path a given drive's own signal chain actually has.
This is why mechanical bodies have no `ampAlpha`-style field of their
own: electrical/analog character lives here, separate from mechanical
properties.

See [JSON config files](build.md#json-config-files) for how an
`analogChain` is specified alongside the mechanical chain in the same
config file.

Every stage's parameters are per **call** of `SimmyAnalogChainStep`, not
per second. Call it more than once per tick (between
`SimmyChainStepDt` substeps) and a `delay` of 1 means one substep, a
`slew` of 100 means 100 per substep, and `noise` is drawn every substep.
Keep the call rate fixed for a given config.

`lag` and `slew` differ in how they handle swings of different size. A
lag takes the same time to cover any swing; a slew limit takes twice as
long for a swing twice as big. An amplifier that reaches full scale from
zero quickly but takes much longer to reverse from -full to +full is a
slew limit, not a lag.
