#!/usr/bin/env python3
"""Indirect FOC truth-vs-estimate and parameter mismatch figures.
Usage: python3 examples/plot_induction_ifoc.py [CSV [OUTPUT_DIR]]
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/induction_ifoc.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row.pop("controller"), []).append({k: float(v) for k, v in row.items()})
names = ["true-flux oracle", "indirect matched", "indirect Rr x0.5", "indirect Rr x1.5"]
colors = ["#555555", "#2675bf", "#d08c13", "#9049a9"]
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})


def values(rows, key):
    return np.array([r[key] for r in rows])


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(out / f"{name}.{ext}", dpi=160)
    plt.close(fig)


fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True, constrained_layout=True)
for name, color in zip(names, colors):
    rows = groups[name]
    time = values(rows, "time_s")
    axes[0].plot(time, values(rows, "speed_rad_s"), color=color, label=name, lw=1.3)
    axes[3].plot(time, values(rows, "current_vector_a"), color=color, lw=1.2)
    if name != "true-flux oracle":
        axes[1].plot(time, values(rows, "true_flux_wb"), color=color, lw=1.3, label=name+" true")
        axes[1].plot(time, values(rows, "control_flux_wb"), color=color, lw=1.1, ls="--")
        error = values(rows, "angle_error_deg")
        error[values(rows, "angle_valid") == 0] = np.nan
        axes[2].plot(time, error, color=color, lw=1.2)
axes[0].plot(time, values(rows, "reference_rad_s"), color="black", ls=":", lw=.8)
axes[0].legend(loc="lower right", ncol=2, fontsize=9)
axes[1].axhline(.5, color="#555555", ls=":", lw=.8)
axes[1].legend(loc="lower right", fontsize=8)
axes[2].axhline(0, color="#555555", ls=":", lw=.8)
axes[3].axhline(8, color="#555555", ls=":", lw=.8)
for ax, label in zip(axes, ["Speed (rad/s)", "Flux linkage (Wb turns)", "Estimated − true angle (°)", "Current vector (A)"]):
    ax.set_ylabel(label)
    ax.axvspan(3.5, 7.5, color="#555555", alpha=.05)
    ax.set_xlim(0, 11)
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Indirect FOC: simulated truth can score what the controller cannot measure\n"
             "Same plant; controller rotor-resistance assumptions differ; 6 Nm brake from 3.5 to 7.5 s", fontsize=13)
fig.supxlabel("Flux: solid = hidden plant truth, dashed = controller estimate. Angle error masked near zero flux.\n"
              "Currents/speed are ideal measurements; this is parameter sensitivity, not hardware validation.", fontsize=9)
save(fig, "induction_ifoc_estimation")

fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
labels = ["True-flux\noracle", "Indirect\nmatched", "Indirect\nRr ×0.5", "Indirect\nRr ×1.5"]
selected = [[r for r in groups[name] if 6.8 < r["time_s"] <= 7] for name in names]
x = np.arange(len(names))
mean = lambda key: np.array([values(rows, key).mean() for rows in selected])
axes[0, 0].bar(x, mean("speed_rad_s"), color=colors)
axes[0, 0].axhline(150, color="gray", ls="--")
axes[0, 0].set(ylabel="Speed (rad/s)", title="Correct speed does not prove correct flux")
for ax, truth, estimate, ylabel, title in [
    (axes[0, 1], "true_flux_wb", "control_flux_wb", "Flux (Wb turns)", "What flux does the controller believe?"),
    (axes[1, 0], "torque_nm", "control_torque_nm", "Torque (Nm)", "Torque inference also depends on the model"),
]:
    ax.bar(x-.18, mean(truth), .36, color=colors, label="plant truth")
    ax.bar(x+.18, mean(estimate), .36, facecolor="none", edgecolor=colors, hatch="//", label="controller model")
    ax.set(ylabel=ylabel, title=title)
    ax.legend(fontsize=9)
axes[1, 1].bar(x, mean("angle_error_deg"), color=colors)
axes[1, 1].axhline(0, color="gray", lw=.8)
axes[1, 1].set(ylabel="Estimated − true angle (°)", title="Flux orientation error under load")
for ax in axes.flat:
    ax.set_xticks(x, labels)
fig.suptitle("A controller can hold speed while believing the wrong flux and torque\n"
             "Loaded window 6.8–7.0 s; plant Rr remains 2 ohm in every run", fontsize=13)
fig.supxlabel("Linear magnetics omit saturation: the Rr ×0.5 overflux result is a warning, not a safe hardware operating point.", fontsize=9)
save(fig, "induction_ifoc_loaded_state")

summary = source.with_name(source.stem + "_summary.csv")
with summary.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["controller", "window_start_s", "window_end_s", "mean_speed_rad_s", "true_flux_wb", "control_flux_wb",
                     "rms_angle_error_deg", "mean_torque_nm", "mean_control_torque_nm", "mean_current_vector_a"])
    for name in names:
        for start, end in [(3.3, 3.5), (6.8, 7), (10.8, 11)]:
            rows = [r for r in groups[name] if start < r["time_s"] <= end]
            valid = [r for r in rows if r["angle_valid"]]
            writer.writerow([name, start, end, values(rows, "speed_rad_s").mean(), values(rows, "true_flux_wb").mean(),
                             values(rows, "control_flux_wb").mean(), np.sqrt(np.mean(values(valid, "angle_error_deg")**2)),
                             values(rows, "torque_nm").mean(), values(rows, "control_torque_nm").mean(),
                             values(rows, "current_vector_a").mean()])
print(f"Wrote indirect FOC estimation and loaded-state figures to {out}; summary: {summary}")
