#!/usr/bin/env python3
"""Fit a simmy chain topology to recorded data, good-enough-quick by
default. See README.md for the full story; this is the CLI.

    fit_chain.py demo --output /tmp/demo
    fit_chain.py fit SPEC.json DATA.csv --output /tmp/fit
    fit_chain.py fit SPEC.json DATA.csv --output /tmp/fit --seed PREVIOUS.json
    fit_chain.py validate FIT.json DATA.csv

A "fit-spec" is a strict superset of simmy's own JSON config: any
bodies[]/couplings[] field may be a plain number (fixed) or
{"guess": g, "min": lo, "max": hi} (free). The fit's plain-number output is
a normal simmy config -- SimmyChainParamsFromJsonFile loads it with zero
translation.
"""

import argparse
import itertools
import json
import sys
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares, minimize_scalar

import chain_bridge

GAP_GRID_POINTS = 7  # "small coarse grid", not axis_compliance.py's 41-point profile.


def _is_free(value):
    return isinstance(value, dict) and "guess" in value


def load_fit_spec(path):
    with open(path) as f:
        spec = json.load(f)
    return load_fit_spec_from_dict(spec)


def load_fit_spec_from_dict(spec):
    free = []
    for i, b in enumerate(spec["bodies"]):
        for field in chain_bridge.BODY_FIELDS:
            v = b.get(field, 0.0)
            if _is_free(v):
                free.append({"kind": "body", "index": i, "field": field, **v})
    for i, c in enumerate(spec.get("couplings", [])):
        for field in chain_bridge.COUPLING_FIELDS:
            v = c.get(field, 0.0)
            if _is_free(v):
                free.append({"kind": "coupling", "index": i, "field": field, **v})
    return spec, free


def materialize(spec, overrides):
    """Plain bodies/couplings lists (every field a float), with `overrides`
    (a {(kind, index, field): value} dict) taking priority over the spec's
    own fixed values, and free-but-unset fields defaulting to 0.0."""
    bodies = []
    for i, b in enumerate(spec["bodies"]):
        body = {}
        for field in chain_bridge.BODY_FIELDS:
            key = ("body", i, field)
            if key in overrides:
                body[field] = overrides[key]
            else:
                v = b.get(field, 0.0)
                body[field] = 0.0 if _is_free(v) else float(v)
        bodies.append(body)
    couplings = []
    for i, c in enumerate(spec.get("couplings", [])):
        coup = {}
        for field in chain_bridge.COUPLING_FIELDS:
            key = ("coupling", i, field)
            if key in overrides:
                coup[field] = overrides[key]
            else:
                v = c.get(field, 0.0)
                coup[field] = 0.0 if _is_free(v) else float(v)
        couplings.append(coup)
    return bodies, couplings


def load_data(csv_path):
    import csv

    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    columns = {}
    for name in rows[0]:
        columns[name] = np.array([float(r[name]) for r in rows], dtype=np.float64)
    return columns


def fixed_initial_positions(encoders, data, body_count):
    """Every body's initial position. A body with an encoder gets its
    first measured sample (known, not fit). A body with no encoder isn't
    directly observed, so its initial position is linearly interpolated
    between its nearest encoder-equipped neighbors by chain index (held at
    the one side's value, or 0.0, if there's no neighbor on the other) --
    a documented simplification, not a fit; see README.md."""
    known = {enc["body"]: float(data[enc["column"]][0]) for enc in encoders}
    positions = [0.0] * body_count
    for b in range(body_count):
        if b in known:
            positions[b] = known[b]
            continue
        left = max((k for k in known if k < b), default=None)
        right = min((k for k in known if k > b), default=None)
        if left is not None and right is not None:
            frac = (b - left) / (right - left)
            positions[b] = known[left] + frac * (known[right] - known[left])
        elif left is not None:
            positions[b] = known[left]
        elif right is not None:
            positions[b] = known[right]
    return positions


def finite_difference_velocity_guess(encoders, data, dt_ticks, body_count):
    """A crude one-sample finite difference per encoder -- fine as a
    least_squares *starting guess* (see initial_velocity_free_params) or
    as the diagnostic-only IC used by `validate`, but NOT accurate enough
    to use as a fixed initial condition during fitting: this chain has no
    restoring force pulling absolute position back to any reference, so
    even a small velocity error there turns into a permanent constant
    position offset for the rest of the replay, large enough to swamp the
    real parameter-fitting signal. That's why the fit itself treats initial
    velocity as a free parameter instead (see initial_velocity_free_params)."""
    guesses = [0.0] * body_count
    for enc in encoders:
        col = data[enc["column"]]
        if len(col) > 1:
            guesses[enc["body"]] = float(col[1] - col[0]) / dt_ticks
    return guesses


def initial_velocity_free_params(body_count, encoders, data, dt_ticks):
    """One free parameter per body for its initial velocity, generously
    bounded from the data's own observed motion -- lets the fit itself
    resolve the unknown initial condition instead of trusting a single
    noisy finite difference (see finite_difference_velocity_guess)."""
    deltas = [
        abs(float(data[enc["column"]][1] - data[enc["column"]][0])) / dt_ticks
        for enc in encoders
        if len(data[enc["column"]]) > 1
    ]
    scale = max(deltas + [1e-6])
    guesses = finite_difference_velocity_guess(encoders, data, dt_ticks, body_count)
    return [
        {
            "kind": "ic_velocity",
            "index": b,
            "field": "velocity",
            "guess": guesses[b],
            "min": -20 * scale,
            "max": 20 * scale,
        }
        for b in range(body_count)
    ]


def initial_position_free_params(body_count, encoders, data):
    """One free parameter per body with NO encoder, for its initial
    position -- centered on the simple neighbor-interpolated guess (see
    fixed_initial_positions) but allowed to move, bounded by the overall
    data span either side of that guess. Fixing an un-encoded body's
    initial position outright (the original, simpler design) forced the
    fit to silently absorb any interpolation error into that body's
    inertia/friction/coupling parameters instead; letting it move on its
    own measurably improved recovery for exactly those parameters. A body
    WITH an encoder keeps its position fixed to the first measured sample
    (exactly known, not worth spending a free parameter on)."""
    known_bodies = {enc["body"] for enc in encoders}
    guesses = fixed_initial_positions(encoders, data, body_count)
    span = max((float(np.ptp(data[enc["column"]])) for enc in encoders), default=1.0)
    span = max(span, 1e-6)
    return [
        {
            "kind": "ic_position",
            "index": b,
            "field": "position",
            "guess": guesses[b],
            "min": guesses[b] - span,
            "max": guesses[b] + span,
        }
        for b in range(body_count)
        if b not in known_bodies
    ]


def build_initial_state(spec, overrides, data, encoders):
    body_count = len(spec["bodies"])
    positions = fixed_initial_positions(encoders, data, body_count)
    known_bodies = {enc["body"] for enc in encoders}
    state = chain_bridge.SimmyChainState()
    for b in range(body_count):
        if b in known_bodies:
            state.bodies[b].position = positions[b]
        else:
            state.bodies[b].position = overrides.get(("ic_position", b, "position"), positions[b])
        state.bodies[b].velocity = overrides.get(("ic_velocity", b, "velocity"), 0.0)
    return state


def make_residual_fn(spec, smooth, fixed_gap_overrides, data, dt_ticks, encoders):
    def residual(x):
        overrides = dict(fixed_gap_overrides)
        for p, val in zip(smooth, x):
            overrides[(p["kind"], p["index"], p["field"])] = val
        bodies, couplings = materialize(spec, overrides)
        params = chain_bridge.make_params(
            bodies,
            couplings,
            feedback_body=spec.get("feedbackBody", 0),
            sub_steps=spec.get("subSteps", 0),
        )
        state = build_initial_state(spec, overrides, data, encoders)
        positions = chain_bridge.simulate(params, state, data["force"], dt_ticks)
        parts = []
        for enc in encoders:
            sim = positions[enc["body"]]
            meas = data[enc["column"]]
            span = max(float(np.ptp(meas)), 1e-9)
            parts.append((sim - meas) / span)
        return np.concatenate(parts)

    return residual


def run_fit_once(spec, free, data, dt_ticks, encoders, x0_overrides=None):
    """One quick pass: a small coarse grid over any free backlashGap
    parameters (refined once, locally, if there's exactly one -- see
    below), a single least_squares call over everything else (the spec's
    free body/coupling params plus one initial-velocity and, for
    un-encoded bodies, initial-position nuisance parameter -- see
    initial_velocity_free_params/initial_position_free_params) per grid
    point. x0_overrides optionally replaces each free param's spec-given
    guess (used by --seed to warm-start from a past solution; it never
    applies to the initial-condition parameters, which are specific to
    this data file, not a property of the chain)."""
    x0_overrides = x0_overrides or {}
    body_count = len(spec["bodies"])
    gaps = [p for p in free if p["field"] == "backlashGap"]
    smooth = [p for p in free if p["field"] != "backlashGap"]
    smooth = (
        smooth
        + initial_velocity_free_params(body_count, encoders, data, dt_ticks)
        + initial_position_free_params(body_count, encoders, data)
    )

    def guess_of(p):
        key = (p["kind"], p["index"], p["field"])
        return x0_overrides.get(key, p["guess"])

    smooth_x0 = np.array([guess_of(p) for p in smooth], dtype=np.float64)
    smooth_lo = np.array([p["min"] for p in smooth], dtype=np.float64)
    smooth_hi = np.array([p["max"] for p in smooth], dtype=np.float64)

    def solve_for_gaps(gap_overrides):
        residual = make_residual_fn(spec, smooth, gap_overrides, data, dt_ticks, encoders)
        if len(smooth) == 0:
            cost = 0.5 * float(np.sum(residual(np.array([])) ** 2))
            return {"x": np.array([]), "cost": cost, "success": True, "message": "no smooth free params"}
        # Scale the robust loss's transition point to this problem's own
        # residual magnitude at the starting guess, instead of a fixed
        # constant: a hardcoded f_scale that happens to be far smaller
        # than the actual residuals flattens the gradient almost
        # everywhere and the fit barely moves from x0 -- measured directly
        # by comparing against plain 'linear' loss on a clean synthetic
        # case, where a mismatched f_scale was the difference between a
        # near-exact recovery and one pinned at a bound.
        r0 = residual(smooth_x0)
        f_scale = max(float(np.median(np.abs(r0))), 1e-6)
        fit = least_squares(
            residual,
            smooth_x0,
            bounds=(smooth_lo, smooth_hi),
            x_scale="jac",
            loss="soft_l1",
            f_scale=f_scale,
            ftol=1e-7,
            xtol=1e-7,
            gtol=1e-7,
            max_nfev=150,
        )
        return {"x": fit.x, "cost": fit.cost, "success": fit.success, "message": fit.message}

    gap_grids = [np.linspace(p["min"], p["max"], GAP_GRID_POINTS) for p in gaps]
    best = None
    best_combo = None
    for combo in itertools.product(*gap_grids) if gaps else [()]:
        gap_overrides = {(p["kind"], p["index"], p["field"]): v for p, v in zip(gaps, combo)}
        result = solve_for_gaps(gap_overrides)
        if best is None or result["cost"] < best["cost"]:
            best = dict(result, gap_overrides=gap_overrides)
            best_combo = combo

    # One local refine, only when there's exactly one free backlash gap --
    # a bounded scalar search around the best grid point's neighbors, the
    # same "profile the coarse grid, then refine" idea
    # ../examples/axis_compliance.py uses, just a single bounded_scalar
    # call instead of its full 41-point profile (two or more simultaneous
    # free gaps stay at the coarse grid's resolution -- a documented
    # limitation, not worth the combinatorial refine search it would take).
    if len(gaps) == 1:
        grid = gap_grids[0]
        idx = int(np.argmin(np.abs(grid - best_combo[0])))
        lo = grid[max(idx - 1, 0)]
        hi = grid[min(idx + 1, len(grid) - 1)]
        if hi > lo:
            gp = gaps[0]

            def scalar_cost(gap_value):
                overrides = {(gp["kind"], gp["index"], gp["field"]): gap_value}
                return solve_for_gaps(overrides)["cost"]

            refined = minimize_scalar(
                scalar_cost, bounds=(lo, hi), method="bounded", options={"xatol": (hi - lo) / 20, "maxiter": 15}
            )
            refined_overrides = {(gp["kind"], gp["index"], gp["field"]): float(refined.x)}
            refined_result = solve_for_gaps(refined_overrides)
            if refined_result["cost"] < best["cost"]:
                best = dict(refined_result, gap_overrides=refined_overrides)

    overrides = dict(best["gap_overrides"])
    for p, val in zip(smooth, best["x"]):
        overrides[(p["kind"], p["index"], p["field"])] = float(val)
    return overrides, best


def validation_rmse(spec, overrides, data, dt_ticks, encoders):
    """Diagnostic only, so this deliberately does NOT re-run the
    initial-velocity fit from run_fit_once -- it seeds from the same crude
    finite difference for every candidate being compared (see
    finite_difference_velocity_guess), which is a fair basis for *ranking*
    candidates against each other even though it understates how well any
    one of them could do with its own best-fit initial condition."""
    bodies, couplings = materialize(spec, overrides)
    params = chain_bridge.make_params(
        bodies, couplings, feedback_body=spec.get("feedbackBody", 0), sub_steps=spec.get("subSteps", 0)
    )
    body_count = len(bodies)
    state = chain_bridge.SimmyChainState()
    positions0 = fixed_initial_positions(encoders, data, body_count)
    velocities0 = finite_difference_velocity_guess(encoders, data, dt_ticks, body_count)
    for b in range(body_count):
        state.bodies[b].position = positions0[b]
        state.bodies[b].velocity = velocities0[b]
    positions = chain_bridge.simulate(params, state, data["force"], dt_ticks)
    out = {}
    for enc in encoders:
        sim = positions[enc["body"]]
        meas = data[enc["column"]]
        out[enc["column"]] = {
            "rmse": float(np.sqrt(np.mean((sim - meas) ** 2))),
            "max_error": float(np.max(np.abs(sim - meas))),
        }
    return out


def build_output_config(spec, overrides):
    bodies, couplings = materialize(spec, overrides)
    return {
        "bodies": bodies,
        "couplings": couplings,
        "feedbackBody": spec.get("feedbackBody", 0),
        "subSteps": spec.get("subSteps", 0),
    }


def cmd_fit(args):
    spec, free = load_fit_spec(args.spec)
    data = load_data(args.data)
    encoders = spec["encoders"]
    dt_ticks = args.dt_ticks

    overrides, fresh_result = run_fit_once(spec, free, data, dt_ticks, encoders)
    report = {
        "schema": "characterization-fitter-chain-v1",
        "spec": str(args.spec),
        "data": str(args.data),
        "dt_ticks": dt_ticks,
        "solver": {
            "success": fresh_result["success"],
            "message": fresh_result["message"],
            "cost": fresh_result["cost"],
        },
        "limitations": [
            "single quick pass: smooth params via one least_squares call, "
            "backlashGap (if free) via a small coarse grid, not a precise profile",
            "no stiction/Coulomb-specific search beyond whatever least_squares finds",
            "see characterization_fitter/README.md for what's deferred to refine-later",
        ],
    }

    if args.seed:
        with open(args.seed) as f:
            seeded_config = json.load(f)
        seed_overrides = {}
        for p in free:
            if p["kind"] == "body":
                seed_overrides[("body", p["index"], p["field"])] = seeded_config["bodies"][p["index"]][p["field"]]
            else:
                seed_overrides[("coupling", p["index"], p["field"])] = seeded_config["couplings"][p["index"]][p["field"]]
        seeded_overrides_result, seeded_result = run_fit_once(
            spec, free, data, dt_ticks, encoders, x0_overrides=seed_overrides
        )
        fresh_rmse = validation_rmse(spec, overrides, data, dt_ticks, encoders)
        seeded_rmse = validation_rmse(spec, seeded_overrides_result, data, dt_ticks, encoders)
        fresh_total = sum(v["rmse"] for v in fresh_rmse.values())
        seeded_total = sum(v["rmse"] for v in seeded_rmse.values())
        chosen_from_seed = seeded_total < fresh_total
        report["refine_comparison"] = {
            "from_past_solution": {"cost": seeded_result["cost"], "validation_rmse": seeded_rmse},
            "from_fresh_guess": {"cost": fresh_result["cost"], "validation_rmse": fresh_rmse},
            "chosen": "from_past_solution" if chosen_from_seed else "from_fresh_guess",
        }
        if chosen_from_seed:
            overrides = seeded_overrides_result

    output_config = build_output_config(spec, overrides)
    args.output.mkdir(parents=True, exist_ok=True)
    config_path = args.output / "fit.json"
    report_path = args.output / "fit-report.json"
    with open(config_path, "w") as f:
        json.dump(output_config, f, indent=2)
    with open(report_path, "w") as f:
        json.dump(report, f, indent=2)
    print(f"wrote {config_path}")
    print(f"wrote {report_path}")


def cmd_validate(args):
    with open(args.fit) as f:
        config = json.load(f)
    data = load_data(args.data)
    with open(args.encoders) as f:
        encoders = json.load(f)["encoders"]
    spec = {"bodies": config["bodies"], "couplings": config["couplings"],
             "feedbackBody": config.get("feedbackBody", 0), "subSteps": config.get("subSteps", 0)}
    overrides = {}
    rmse = validation_rmse(spec, overrides, data, args.dt_ticks, encoders)
    print(json.dumps(rmse, indent=2))


def cmd_demo(args):
    """Generates synthetic ground-truth data from a real SimmyChainParams
    via the same bridge the fitter uses -- a self-test fixture, not a
    claim about any real machine (same three_mass_leadscrew.json ratios)."""
    true_bodies = [
        {"inertia": 64, "friction": 0.02},
        {"inertia": 12, "friction": 0.01},
        {"inertia": 8, "friction": 0.01, "stictionLimit": 80, "coulombLevel": 50},
    ]
    true_couplings = [
        {"springK": 1.69, "springD": 5.0, "backlashGap": 0.1},
        {"springK": 8.0, "springD": 0.5, "backlashGap": 2.0},
    ]
    params = chain_bridge.make_params(true_bodies, true_couplings, feedback_body=0, sub_steps=20)
    state = chain_bridge.SimmyChainState()

    rng = np.random.default_rng(0)
    steps = args.steps
    force = np.zeros(steps)
    t = 0
    level = 0.0
    while t < steps:
        level = rng.uniform(-60, 60)
        hold = rng.integers(200, 600)
        force[t : t + hold] = level
        t += hold
    positions = chain_bridge.simulate(params, state, force, 1.0)

    args.output.mkdir(parents=True, exist_ok=True)
    data_path = args.output / "demo_data.csv"
    with open(data_path, "w") as f:
        f.write("time,force,motor_position,table_position\n")
        for i in range(steps):
            f.write(f"{i},{force[i]},{positions[0, i]},{positions[2, i]}\n")

    guess_spec = {
        "bodies": [
            {"inertia": {"guess": 50, "min": 1, "max": 500}, "friction": {"guess": 0.05, "min": 0, "max": 1}},
            {"inertia": {"guess": 10, "min": 1, "max": 500}, "friction": {"guess": 0.02, "min": 0, "max": 1}},
            {"inertia": 8, "friction": 0.01, "stictionLimit": 80, "coulombLevel": 50},
        ],
        "couplings": [
            {"springK": {"guess": 1, "min": 0.1, "max": 50}, "springD": {"guess": 2, "min": 0, "max": 20}},
            {
                "springK": {"guess": 5, "min": 0.1, "max": 50},
                "springD": {"guess": 1, "min": 0, "max": 20},
                "backlashGap": {"guess": 1, "min": 0, "max": 5},
            },
        ],
        "encoders": [
            {"column": "motor_position", "body": 0},
            {"column": "table_position", "body": 2},
        ],
        "feedbackBody": 0,
        "subSteps": 20,
    }
    spec_path = args.output / "demo_spec.json"
    with open(spec_path, "w") as f:
        json.dump(guess_spec, f, indent=2)

    print(f"wrote {data_path}")
    print(f"wrote {spec_path}")
    print(f"ground truth: bodies={true_bodies} couplings={true_couplings}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p_demo = sub.add_parser("demo", help="generate synthetic ground-truth data + a fit-spec")
    p_demo.add_argument("--output", type=Path, required=True)
    p_demo.add_argument("--steps", type=int, default=20000)
    p_demo.set_defaults(func=cmd_demo)

    p_fit = sub.add_parser("fit", help="fit a spec against recorded data")
    p_fit.add_argument("spec", type=Path)
    p_fit.add_argument("data", type=Path)
    p_fit.add_argument("--output", type=Path, required=True)
    p_fit.add_argument("--dt-ticks", type=float, default=1.0)
    p_fit.add_argument("--seed", type=Path, default=None, help="previous fit's output config, for the refine comparison")
    p_fit.set_defaults(func=cmd_fit)

    p_val = sub.add_parser("validate", help="replay a fitted config open-loop against held-out data")
    p_val.add_argument("fit", type=Path)
    p_val.add_argument("data", type=Path)
    p_val.add_argument("encoders", type=Path, help="a fit-spec JSON, read only for its encoders[] list")
    p_val.add_argument("--dt-ticks", type=float, default=1.0)
    p_val.set_defaults(func=cmd_validate)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    sys.exit(main())
