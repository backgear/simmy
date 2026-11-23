# Driving simmy with your own controller

simmy has no opinion about control algorithms. You drive it with a plain
function pointer, which may be a three-line demo P loop or a real
production control cascade linked in from somewhere else.

## Controller convention

For the position/axis simulation harness, the controller callback is:

```c
typedef int32_t (*SimmyControllerFn)(void *userData, int32_t targetPosition,
                                      int32_t actualPosition, int32_t actualVelocity,
                                      uint32_t dtCycles);
```

Motor experiments separately schedule voltage/gate controllers at
explicit sample rates (see [loop frequencies](loop-frequencies.md)); they
need not squeeze vector electrical commands through this scalar
position/force callback. Both styles keep controllers outside plant
physics.

The signature is chosen to match a typical fixed-point position-control
tick function closely enough that wrapping a real controller is a direct
passthrough, not an adapter fighting the API -- see
`examples/analog_servo_integration/main.c`, where the entire adapter for
a real production control cascade is:

```c
static int32_t ControlAdapter(void *userData, int32_t targetPosition, int32_t actualPosition,
                               int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  return ControlTick(targetPosition, actualPosition, actualVelocity, dtCycles);
}
```

The plain examples' own demo PD controller is just another ordinary
implementation of the same typedef -- no fallback/shortcut version.

## Closing the loop

Closing the control loop is assumed to happen against **one body** in
the chain, `SimmyChainParams.feedbackBody` -- intended to be the body a
real motor-mounted encoder would read. Reading position or velocity off
any **other** body (e.g. a table at the far end of a lead screw,
standing in for where a linear scale would sit) is fully supported for
**measurement/evaluation** -- precisely so you can judge whether
motor-side-only control tracks acceptably at the load, which is the
comparison that matters for judging whether a config fits a real
physical setup. simmy does not support closing the loop against a
non-feedback body; that's a deliberate scope boundary (see
[scope and non-goals](scope.md)).

See [examples](examples.md) for both the plain demo-controller examples
and the real-firmware integration examples (`analog_servo_integration`,
`rls_integration`) that exercise this convention against actual
production control code.
