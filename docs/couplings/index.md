# Couplings

A `SimmyCouplingParams` connects two adjacent bodies in the chain with
one spring/damper/backlash law -- the same struct regardless of whether
the real thing between them is a flexible shaft, a leadscrew/nut, a
belt, a gear mesh, or a friction clutch. What distinguishes them is
entirely in which numbers you compute and hand over; simmy has no idea
which kind of transmission it is.

The one fact that drives all of it: the coupling computes a single
relative displacement between the two bodies' positions and applies the
identical spring/damper force, with opposite sign, to each
(`src/simmy.c`'s `ChainStep`). There's no transformer/ratio node in
that graph -- it's a direct 1:1 connection along one shared axis.

[Shaft couplings](shaft-couplings.md) (rigid, spider/jaw, helical) *are*
that direct 1:1 connection -- no ratio, no reflection needed. Any real
transmission with its own ratio (a gear pair, a belt-and-pulley, a
leadscrew's lead) has to be **fully reflected onto that one shared axis
before it reaches simmy** -- see [Gears](gears.md) for the reflection
math (inertia and stiffness shrink by the ratio squared; backlash
grows by the ratio, the opposite direction) and [Belts](belts.md) and
[Clutches](clutches.md) for the transmissions that also need a force
ceiling (`forceLimit`) rather than just reflected inertia/stiffness.

## Reflected inertia compared

| Transmission | Reflects by | Where stiffness comes from | Where backlash comes from |
|---|---|---|---|
| [Gear pair](gears.md) | `J_load / N^2` | tooth-mesh torsion (often treated as rigid) | tooth clearance; worsens with wear/center-distance drift -- caller supplies it |
| [Leadscrew](#leadscrews) | `m * (L / 2*pi)^2` | screw/nut assembly stiffness | nut-to-screw axial play, usually spec'd directly in mm |
| [Timing belt](belts.md) | `J_load / N^2` (N from pulley diameters) | belt span elasticity (`E*A/L_span`), not tooth torsion | tooth/tensioner slack -- small, and doesn't grow with "distance" the way a gear mesh does |
| [V/poly-V belt](belts.md) | same as timing belt | same elastic-span term, below the slip limit | friction drive has no real dead zone -- it has a torque *ceiling* instead (`forceLimit`) |

### Leadscrews

A leadscrew reflects a translating mass the same way a gear reflects a
rotating one, just through its lead `L` (linear travel per input
revolution) instead of a tooth ratio:

```
v = omega * L / (2*pi)
=>  J_reflected = m * (L / (2*pi))^2
```

Leadscrew backlash is usually already spec'd as a **linear** play at the
nut (mm), which is often more convenient than a gear's angular spec --
convert it to the shared axis with the same lead relationship:
`theta_b_reflected = backlash_mm * (2*pi / L)`.
`examples/three_mass_leadscrew.c` models exactly this stage.

## Mutating params mid-run: a worked example (not a recommendation)

Several "not supported" gaps on [scope](../scope.md#what-a-few-specific-models-assume)
and [clutches](clutches.md) turn out not to need a new simmy feature at
all, because the caller already owns `SimmyChainParams` and hands it to
`SimmyChainStep` fresh every tick -- nothing stops you from changing a
coupling's numbers between calls, based on whatever you've computed
about the current state. Topologically nothing changes: it's still the
same two bodies, the same one `SimmyCouplingParams`, the same single
relative-displacement term in `ChainStep` -- the only thing moving is
which numbers are sitting in that struct when `ChainStep` reads them.

A minimal, deliberately-not-recommended worked example: a **floppy
load** -- a cable, string, or slack chain segment that can only pull,
never push. `SimmyCouplingParams` has no sign-dependent stiffness, so
there's nothing to set once; instead, read the current relative
position back every tick and decide which regime you're in before
stepping:

```c
int32_t rel = SimmyChainBodyPositionCounts(&state, 1)
            - SimmyChainBodyPositionCounts(&state, 0);
if (rel > 0) {
  /* cable taut -- pulling */
  params.couplings[0].springK = cableK;
  params.couplings[0].springD = cableD;
} else {
  /* slack -- floppy, no coupling force at all */
  params.couplings[0].springK = 0.0;
  params.couplings[0].springD = 0.0;
}
SimmyChainStep(&state, &params, force);
```

Why this isn't the recommended path, even though it works: there's no
hysteresis in that `if`, so a system that dithers right around
`rel == 0` reintroduces, by hand, the exact chatter that deciding
[stiction](../scope.md#what-a-few-specific-models-assume) before
integrating exists to avoid. It also makes you duplicate, in your own
code, the same relative-position definition `ChainStep` already
computes privately, with the same sign convention you have to get
right yourself. And it only reaches as far as the params struct goes --
it can't expose the force `ChainStep` actually applied, so there's
still no heat/wear ledger this way, and it can't add a degree of
freedom, so off-axis effects and a real time-varying ratio (see
[universal joints](shaft-couplings.md#universal-joints)) stay out of
reach. Most of the clutch "modes" in [clutches](clutches.md) are
reachable this way, if you're willing to own that bookkeeping yourself.

A **one-way (sprag) clutch** is the same floppy-load pattern exactly,
no new code shape needed -- swap the `if` from the sign of relative
*position* to the sign of relative *velocity*: `forceLimit` (or
`springK`/`springD`) set to the real locked-direction values when
`relVel` has the engaging sign, and to 0 (freewheeling) when it
reverses. Still stateless, still one branch, still read every tick.

A **ball-detent clutch** is the same pattern with one real addition: it
needs a stored mode bit, not just a stateless `if` on the current
state. Ball-detent engagement and release happen at two different
torque thresholds (hysteresis, so it doesn't chatter right at the
engagement point the way the floppy load's bare `if` can) -- so your
mutate-before-`SimmyChainStep` code has to carry that bit across ticks
itself, alongside whatever `forceLimit` it's currently setting. And
the actual detent physics -- contact force between ball and ramp, spring
preload, ramp angle -- is on the caller the same way `cableK`/`cableD`
were above: simmy only ever sees the `forceLimit`/`springK` numbers you
hand it, never how you got them.
