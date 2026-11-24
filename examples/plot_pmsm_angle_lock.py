#!/usr/bin/env python3
"""Plot the forced-alignment / angle-lock torque-pulse position response
across swept rotor inertia, overlaid on one time axis, to show the
textbook period ~ sqrt(J) scaling and common equilibrium that validate
the PM plant's own electromechanical response (no controller involved).
Usage: python3 examples/plot_pmsm_angle_lock.py [CSV [OUTPUT_DIR]]
NumPy/Matplotlib are plotting-only dependencies.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/pmsm_angle_lock.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(float(row.pop("inertia_kgm2")), []).append(
            {k: float(v) for k, v in row.items()})
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})
inertias = sorted(groups)
colors = plt.cm.viridis(np.linspace(0.15, 0.85, len(inertias)))


def values(rows, key):
    return np.array([r[key] for r in rows])


def first_period(rows):
    """Time between the first two angle==0 (falling) crossings, a proxy
    for the damped oscillation's own period -- same measure the unit
    test uses, so this annotation and the test agree."""
    time, angle = values(rows, "time_s"), values(rows, "mechanical_angle_rad")
    crossings = []
    for i in range(1, len(angle)):
        if angle[i-1] > 0 >= angle[i]:
            crossings.append(time[i])
            if len(crossings) == 2:
                return crossings[1]-crossings[0]
    return None


fig, axes = plt.subplots(2, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
periods = {}
for inertia, color in zip(inertias, colors):
    rows = groups[inertia]
    time = values(rows, "time_s")
    label = f"J={inertia:g} kg*m^2"
    periods[inertia] = first_period(rows)
    if periods[inertia] is not None:
        label += f" (period {periods[inertia]*1000:.0f} ms)"
    axes[0].plot(time, values(rows, "mechanical_angle_rad"), color=color, label=label, lw=1.3)
    axes[1].plot(time, values(rows, "current_a"), color=color, lw=1)
axes[0].axhline(0, color="#555555", ls=":", label="equilibrium (fixed drive angle)")
axes[0].legend(loc="upper right", fontsize=9)
axes[0].set_ylabel("Rotor mechanical angle (rad)")
axes[1].set_ylabel("Stator current vector (A)")
axes[1].set_xlabel("Simulated time (s)")
fig.suptitle("Forced alignment on a FIXED stator voltage vector: same restoring torque,\n"
             "different rotor inertia -- period grows with sqrt(J), same equilibrium for all",
             fontsize=13)
base = min(inertias)
ratios = ", ".join(f"{j/base:.0f}x J->{np.sqrt(j/base):.2f}x period" for j in inertias[1:])
fig.supxlabel(f"No controller, no feedback, no added mechanical friction -- validates the PLANT's own electromechanical response.\n"
              f"Predicted vs. J={base:g}: {ratios} (sqrt-J scaling); see the legend for each run's own measured period.", fontsize=8.5)
for ext in ("png", "svg"):
    fig.savefig(out/f"pmsm_angle_lock_settling.{ext}", dpi=160)
plt.close(fig)

summary = source.with_name(source.stem + "_summary.csv")
with summary.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["inertia_kgm2", "measured_period_s", "sqrt_j_ratio_vs_smallest",
                     "measured_period_ratio_vs_smallest"])
    base_period = periods[base]
    for inertia in inertias:
        p = periods[inertia]
        writer.writerow([inertia, p, np.sqrt(inertia/base),
                         p/base_period if p and base_period else ""])
print(f"Wrote angle-lock settling figure to {out}; summary: {summary}")
