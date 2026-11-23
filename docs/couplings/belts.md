# Belts

Belts split into two families that look similar on the shelf and model
very differently: positive-engagement (timing) belts, and friction
belts (V-belt, poly-V/multi-rib). The difference isn't cosmetic -- it's
the difference between "fits `SimmyCouplingParams` cleanly" and "needs
a nonlinearity simmy doesn't have yet."

## Timing belts

A timing belt is a positive drive, same as a gear pair -- the teeth
can't skip under normal operation, so the two pulleys stay in exact
ratio (GT2 and HTD both included). [Reflect inertia](gears.md#reflecting-inertia)
the identical way, using the pulley-diameter (or tooth-count) ratio in
place of a gear ratio: `J_reflected = J_load / N^2`.

That "exact ratio" is an assumption about the belt, not something
simmy checks. It holds for any positive-engagement drive with round
pulleys or sprockets, roller chain included -- but it isn't universal:
a chain develops a real periodic ripple as each link straightens onto
the sprocket (the "chordal" or "polygon" effect, worse with fewer
sprocket teeth) that a constant-ratio model like this one can't
represent. [Universal joints](shaft-couplings.md#universal-joints) are
the same gap in a different transmission, pushed much further -- there
the non-constant ratio isn't a small correction, it's the entire
reason the (non-CV) joint behaves the way it does.

Where it differs from a gear is the *source* of stiffness and backlash:

- **Stiffness** comes from the belt's own elastic stretch along the
  taut span, not tooth-mesh torsion: `springK` scales with `E*A /
  L_span` (effective tensile modulus times cross-sectional/cord area,
  over the unsupported span length between pulleys). A longer span is a
  softer coupling -- this is the main lever you have over a timing
  belt's effective `springK`, more than anything about the teeth
  themselves.
- **Backlash** is small and comes from tooth-to-tooth clearance plus
  tensioner slack, not from "mesh distance." Unlike a gear pair, a belt
  doesn't develop growing clearance as center distance drifts -- you
  retension it instead. That makes the backlash side of this coupling
  comparatively easy to bound; most of the modeling effort belongs on
  `springK`/`springD` from the span.

## V-belts and poly-V (multi-rib) belts

These are friction drives: torque transmits through friction between
the belt and the pulley sidewalls (V-belt) or ribs (poly-V), not a
positive tooth lock.

Reflected inertia is identical to the timing-belt case (same
`J_load / N^2` from the pulley ratio), and the belt's own elastic
stretch under tension still gives you a legitimate `springK`, exactly
as above. What's different is that a friction drive also has a hard
ceiling on transmissible torque, set by installed belt tension, the
pulley wrap angle, and the friction coefficient -- and `SimmyCouplingParams`
now has a field for exactly that: `forceLimit` clamps
`springK*rel + springD*relVel` to `+-forceLimit` every tick, the same
way `stictionLimit`/`coulombLevel` cap a body's own friction. 0 (the
default) leaves a coupling unlimited, so existing configs are
unaffected.

The underlying relationship for where to set that ceiling is the
capstan (Euler-Eytelwein) equation: at the verge of gross slip, the
tight- and slack-side tensions on the critical pulley (the one with the
smaller wrap angle, usually the driven pulley) are related by

```
T_tight / T_slack = e^(mu * theta)      (theta in radians)
```

For a simple two-pulley loop with a fixed center distance (no separate
dynamic tensioner), total belt tension is roughly conserved as load
ramps up: `T_tight + T_slack ~= 2 * T_static`, where `T_static` is
whatever tension the belt was installed at. Combining the two and
solving for the torque at which `T_slack` would have to hit zero gives
a closed form for the max transmissible torque at radius `r`:

```
T_max = 2 * T_static * r * tanh(mu * theta / 2)
```

Convert that `T_max` (a torque, at the belt's pulley radius) into the
chain's internal force units the same way you would any other force
(see [units](../mechanical-chain.md#units-and-numerical-resolution)),
and set `forceLimit` to it. Below `T_max` the belt behaves like the
timing-belt case above (no real dead zone, since it stays in tension);
*above* it, it doesn't develop more lash the way a loose gear mesh
would -- it slips continuously, with the excess force simply discarded
rather than transmitted, and no return to lockstep until load drops
back under the ceiling. A V-belt's wedge-shaped groove multiplies the
effective friction available for the same installed tension --
substitute `mu' = mu / sin(beta / 2)` for `mu` above, where `beta` is
the groove's included angle (commonly 34-38 degrees). That wedging
effect is *why* V-belts carry more torque than a round belt of similar
size and tension -- see [O-ring and round belts](#o-ring-and-round-belts)
below.

What `forceLimit` does *not* give you: there's no slip-heat or wear
ledger (the discarded force just vanishes, the way clamping any other
force in simmy already works -- it isn't converted to a joule count
anywhere), and it's a constant ceiling, not a speed-dependent one.
[Clutches](clutches.md) goes through which clutch types that second gap
rules out.

## O-ring and round belts

A round (O-ring) belt is the same friction-drive physics as a V-belt,
run on a round or shallow groove instead of a wedge -- and it really is
simpler to model, in a specific way: there's no wedge angle, so the
plain capstan equation applies with the belt's *actual* friction
coefficient, not an amplified `mu' = mu / sin(beta/2)`. One fewer
parameter, and one less place to get the geometry wrong. The cost is
exactly what you'd expect from dropping the wedge multiplier: for the
same installed tension and pulley size, an O-ring belt's `T_max` is
lower than a V-belt's -- which is why round belts show up on light
loads (printers, small rollers, tape mechanisms) and V/poly-V belts
take over once real torque is involved.

They're also usually a single molded elastomer loop rather than a
cord-reinforced composite, so `springK` from span elasticity is a
single material's modulus rather than a cord-plus-rubber estimate --
one less thing to guess at than with a V-belt or timing belt, though
still the same `E*A/L_span` form. Reflected inertia is the same pulley-
ratio formula as every other belt here.
