#!/usr/bin/env python3
"""Plot the forward abrupt-start and isolated-stop slip regression traces.
Usage: build/test_stepper_moves --csv build/stepper_moves.csv
       python3 examples/plot_stepper_moves.py [CSV [OUTPUT_PREFIX]]
Requires numpy/matplotlib only for plotting.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/stepper_moves.csv")
prefix = Path(sys.argv[2] if len(sys.argv) > 2 else "build/stepper_start_stop_slip")
prefix.parent.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        if row["scenario"] not in ("fast start forward", "hard stop forward"):
            continue
        groups.setdefault(row["scenario"], []).append(row)

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(2, 2, figsize=(12, 7.5), sharex="col", constrained_layout=True)
for col, name in enumerate(("fast start forward", "hard stop forward")):
    rows = groups[name]
    time = np.array([float(r["time_s"]) for r in rows])
    command = np.array([float(r["command_rad"]) for r in rows])
    theta = np.array([float(r["position_rad"]) for r in rows])
    velocity = np.array([float(r["velocity_rad_s"]) for r in rows])
    error = np.array([float(r["error_cycles"]) for r in rows])
    axes[0, col].plot(time, command, color="#555555", ls="--", label="commanded field / N")
    axes[0, col].plot(time, theta, color="#2675bf", label="actual rotor position")
    axes[0, col].set(ylabel="Unwrapped position (rad)",
                     title="Abrupt start from rest" if col == 0 else "Abrupt stop from synchronism")
    axes[0, col].legend(loc="upper right", fontsize=9)
    axes[0, col].text(.98, .08, f"Settled error: {error[-1]:+.0f} electrical cycles\n"
                     f"({4*error[-1]:+.0f} full steps)", transform=axes[0, col].transAxes,
                     ha="right", va="bottom", bbox={"facecolor": "white", "alpha": .85, "edgecolor": "none"})
    axes[1, col].plot(time, velocity, color="#2675bf", label="rotor velocity")
    axes[1, col].axhline(0, color="gray", lw=.8)
    axes[1, col].set(ylabel="Rotor velocity (rad/s)", xlabel="Simulated time (s)")
    axes[1, col].legend(loc="upper right", fontsize=9)
    if col == 0:
        end = next(float(r["time_s"]) for r in rows if r["phase"] == "hold")
        for ax in axes[:, col]:
            ax.axvline(end, color="#d08c13", ls=":", lw=1.3)
        axes[1, col].text(.98, .08, f"Pulse train ends at ≈ {end:.3f} s\nFinal currents remain energized",
                         transform=axes[1, col].transAxes, ha="right", va="bottom", fontsize=9)
    else:
        axes[1, col].text(.98, .15, "Command frozen at t = 0\nRotor begins at 20 rad/s, not rest",
                         transform=axes[1, col].transAxes, ha="right", va="bottom", fontsize=9)
fig.suptitle("The same torque law permits slips at both ends of a move\n"
             "Actual rotor dynamics, not scripted missed steps; correct wrapped alignment can hide position loss", fontsize=13)
fig.supxlabel("Ideal-current demonstration only: no load, detent, or drive errors; no voltage/back-EMF limits.\n"
              "20 rad/s is a deliberately severe model test, not a calibrated real-motor threshold.", fontsize=9)
for extension in ("png", "svg"):
    fig.savefig(f"{prefix}.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {prefix}.{{png,svg}}")
