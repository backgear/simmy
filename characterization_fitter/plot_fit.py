#!/usr/bin/env python3
"""Plot measured vs. simulated position per encoder for a fitted config
(Matplotlib, optional dependency -- same convention as
../examples/plot_axis_identification.py). Diagnostic only: shows how bad
(or good) a fit is, it doesn't change anything.

    plot_fit.py FIT.json DATA.csv SPEC.json --output plot.png
"""

import argparse
import json
from pathlib import Path

import numpy as np

import chain_bridge
import fit_chain as fc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("fit", type=Path)
    parser.add_argument("data", type=Path)
    parser.add_argument("spec", type=Path, help="a fit-spec JSON, read only for its encoders[] list")
    parser.add_argument("--dt-ticks", type=float, default=1.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default=None)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(args.fit) as f:
        config = json.load(f)
    with open(args.spec) as f:
        encoders = json.load(f)["encoders"]
    data = fc.load_data(args.data)

    bodies, couplings = config["bodies"], config["couplings"]
    params = chain_bridge.make_params(
        bodies, couplings, feedback_body=config.get("feedbackBody", 0), sub_steps=config.get("subSteps", 0)
    )
    body_count = len(bodies)
    pos0 = fc.fixed_initial_positions(encoders, data, body_count)
    vel0 = fc.finite_difference_velocity_guess(encoders, data, args.dt_ticks, body_count)
    state = chain_bridge.SimmyChainState()
    for b in range(body_count):
        state.bodies[b].position = pos0[b]
        state.bodies[b].velocity = vel0[b]
    positions = chain_bridge.simulate(params, state, data["force"], args.dt_ticks)

    t = data["time"] if "time" in data else np.arange(len(data["force"]))
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True, sharex=True)
    colors = plt.rcParams["axes.prop_cycle"].by_key()["color"]
    for enc, color in zip(encoders, colors):
        meas = data[enc["column"]]
        sim = positions[enc["body"]]
        span = max(float(np.ptp(meas)), 1e-9)
        axes[0].plot(t, meas, color=color, label=f"{enc['column']} measured")
        axes[0].plot(t, sim, "--", color=color, label=f"{enc['column']} simulated")
        axes[1].plot(t, (sim - meas) / span * 100, color=color, label=enc["column"])
    axes[0].set(title=args.title or "Fit vs. measured", ylabel="Position [counts]")
    axes[1].set(xlabel="Tick", ylabel="(simulated - measured) / span [%]")
    for ax in axes:
        ax.grid(alpha=0.3)
        ax.legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
