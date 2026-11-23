# Gears

A gear mesh is a `SimmyCouplingParams` like any other -- the same
spring/damper/backlash machinery used for a flexible shaft or a belt.
What makes it *a gear* is entirely in the numbers you put into that
struct; simmy has no `gearRatio` field and no idea a ratio exists.

That matters because of how the coupling actually works
(`src/simmy.c`'s `ChainStep`): it computes one relative displacement
between the two adjacent bodies' positions and applies the identical
spring/damper force, with opposite sign, to each. There's no
transformer in that graph -- it's a direct 1:1 connection along a single
shared axis. A real N:1 gear pair has to be **fully reflected onto that
one shared axis before it ever reaches simmy.**

## Reflecting inertia

Take `N` as the reduction ratio, input speed over output speed (`N > 1`
for a reducer -- the output turns slower). Energy has to match on both
sides of the mesh:

```
(1/2) J_reflected * omega_in^2 = (1/2) J_load * omega_out^2
omega_out = omega_in / N
=>  J_reflected = J_load / N^2
```

A 10:1 reducer makes a load's inertia look 100x smaller from the motor
side -- this is why gearboxes are also inertia-matching tools, not just
torque multipliers. The same N^2 applies to any torsional stiffness you
know at the mesh itself: a real mesh stiffness `K_mesh` (Nm/rad)
reflects to `K_mesh / N^2` when used as `springK` on the input-referred
axis.

## Reflecting backlash -- the opposite direction

Backlash is a geometric clearance, not an energy quantity, so it does
**not** follow the same N^2 law. A fixed angular play at the mesh,
`theta_b` (measured as how far the output can rock with the input held
still), corresponds to this many turns of the faster-moving input shaft:

```
theta_b_reflected = N * theta_b
```

Backlash *grows* linearly with N when viewed from the input side -- the
opposite direction from inertia and stiffness, and only linear, not
squared. This is exactly why backlash near the motor end of a
multi-stage gear train matters far more than the same physical play at
the very last mesh before the load: everything downstream of a sloppy
early stage amplifies it by whatever ratio remains.

And this is the part simmy cannot do for you: it has no idea what gear
distance, tooth wear, or mesh quality you're modeling. As center
distance drifts from nominal or teeth wear, the real backlash at that
mesh grows -- you have to look that number up (or measure it) and feed
in the already-reflected value yourself. `backlashGap` just enforces
whatever dead zone you hand it.

## Recipe

To add a gear-driven body downstream of an existing one in the chain:

1. Keep every body on the chain's one shared counts-per-radian scale
   (see [units](../mechanical-chain.md#units-and-numerical-resolution)) --
   as if both bodies sat on the same physical shaft. A single external
   gear pair (or a crossed belt) reverses direction, and simmy has no
   sign for that either -- for an inverting mesh, flip which physical
   rotation direction you call "positive" on the downstream body, so
   that the two bodies' positions still agree at the real rigid-mesh
   equilibrium.
2. `inertia` for the new body: real load inertia / `N^2`, then convert
   to internal units the normal way.
3. `springK`/`springD`: real mesh stiffness/damping / `N^2`, if you have
   it -- in practice a gear mesh is usually treated as near-rigid, with
   backlash doing the real modeling work.
4. `backlashGap`: real mesh backlash (arc-minutes or radians, from a
   spec sheet or measurement) * `N`, then converted to counts.

See [Couplings](index.md#reflected-inertia-compared) for how this
reflection compares to a leadscrew's or a belt's.
