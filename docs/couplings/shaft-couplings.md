# Shaft couplings

Rigid, spider/jaw, and helical couplings all connect two shafts
directly (no ratio, no slip mode) -- they're the simplest category on
this page, each a plain instance of `SimmyCouplingParams` with no
reflection math and (apart from backlash) no real nonlinearity to
flag.

## Rigid

No compliance by design. If this is truly the only thing between the
motor and whatever reads position back -- a sleeve or set-screw
coupling joining a motor shaft directly to an encoder, say -- you don't
even need a second body: a **1-body chain is already exactly a rigid
coupling** (see [mechanical chain](../mechanical-chain.md)), no spring
math at all, which is both correct and the numerically safest choice.
Only add a second body if you want to deliberately model some real
compliance (even a "rigid" coupling has finite stiffness) or clearance
(a loose set screw, fretting).

## Spider / jaw (elastomer insert)

Two hubs with interleaved jaws and a soft elastomer spider insert
between them, absorbing shock and some misalignment -- and some
backlash, once the insert wears or if it's an undersized insert (some
jaw couplings are built with deliberate looseness to avoid binding).
`springK`/`springD` come directly from the insert's shear
stiffness/damping; `backlashGap` from the jaw-to-insert clearance,
which grows as the insert wears or takes a compression set -- the same
"caller supplies the real number, it's not computed" story as a gear
mesh's backlash, just driven by elastomer wear instead of metal wear.

`examples/three_mass_leadscrew.c`'s first coupling
(`springK=1.69, springD=5.0, backlashGap=0.1`) is labeled a belt stage
in that example's own comment, not a spider coupling -- but the exact
same numbers are what you'd use for a direct motor-to-screw spider
coupling instead: same math, and no ratio to reflect either way, since
both ends turn 1:1.

## Helical / beam

Machined from a single piece of metal with a helical cut, giving
flexibility for angular and parallel shaft misalignment while staying
torsionally fairly stiff. In torsion -- the only axis the chain
simulates -- it's just a goofy spring: the same `springK`/`springD`
representation as a spider coupling, usually with negligible backlash
(it's a single machined part, no separate pieces to develop clearance
between).

What we don't model: **off-axis compliance is the entire point of a
helical coupling** -- that's the reason you'd pick one over a rigid
coupling in the first place. simmy's chain is a one-dimensional
torsional model; the helical cut's actual job happens in a direction
this library doesn't represent at all. Worth stating plainly as a
boundary, the same as [the stepper model](../motors/stepper.md) being
upfront about what it leaves out.

## Universal joints

A classic (non-CV) universal joint doesn't hold a constant velocity
ratio, even at a fixed shaft angle and fixed input speed -- the
instantaneous ratio varies with rotation angle, so the output
oscillates twice per revolution around the mean 1:1 ratio instead of
tracking it exactly. simmy's coupling has no model for that at all:
its one relative-displacement term is a true constant-ratio
connection, every tick, the same assumption [belts](belts.md) and
gears make.

Two things keep that from mattering as often as it sounds like it
should. A **CV (constant-velocity) joint** -- the kind in every
front-wheel-drive halfshaft -- is built specifically to cancel this
ripple by its geometry, so the gap is a non-issue there by
construction, not by approximation. And a classic **double-U-joint
driveshaft** (behind a rear-wheel-drive car, or a machine tool's long
coupling) is normally built with the two joints phased so the second
joint's ripple cancels the first's -- correctly phased, the ratio at
the far end is constant again, same as a CV joint, and only a single
joint or an improperly phased pair actually shows the oscillation this
library leaves out.
