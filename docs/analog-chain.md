# The analog signal chain

A composable "train of analog type values" standing in for a real
DAC -> amplifier -> motor signal path: an ordered list of simple stages
(`gain`, `saturate`, `quantize` -- a DAC's LSB step, `lag` -- one-pole
electrical bandwidth, `noise`), each transforming a single scalar value.
Nothing requires using all of these or in any particular order --
compose whatever path a given drive's own signal chain actually has.
This is why mechanical bodies have no `ampAlpha`-style field of their
own: electrical/analog character lives here, separate from mechanical
properties.

See [JSON config files](build.md#json-config-files) for how an
`analogChain` is specified alongside the mechanical chain in the same
config file.
