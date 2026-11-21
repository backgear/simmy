#!/usr/bin/env python3
"""Plot ideal true-flux FOC versus V/Hz and a separate voltage-limit stress.
Usage: python3 examples/plot_induction_foc.py [CSV [OUTPUT_DIR]]
NumPy/Matplotlib are plotting-only dependencies.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/induction_foc.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row.pop("drive"), []).append({k: float(v) for k, v in row.items()})
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})
colors = {"open-loop VHz": "#d08c13", "ideal flux FOC": "#2675bf"}
voltage_limit = 230*np.sqrt(2/3)


def values(rows, key):
    return np.array([r[key] for r in rows])


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(out / f"{name}.{ext}", dpi=160)
    plt.close(fig)


fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
for name, color in colors.items():
    rows = groups[name]
    time = values(rows, "time_s")
    for ax, key in zip(axes, ["rotor_rad_s", "current_vector_a", "rotor_flux_wb", "torque_nm"]):
        ax.plot(time, values(rows, key), color=color, label=name, lw=1.3)
rows = groups["ideal flux FOC"]
axes[0].plot(time, values(rows, "speed_reference_rad_s"), color="#555555", ls="--", label="speed request / VHz field speed")
axes[0].legend(loc="lower right", fontsize=9)
axes[1].axhline(8, color="#555555", ls=":", label="FOC current-reference limit")
axes[1].legend(loc="upper left", fontsize=9)
axes[2].axhline(.5, color="#555555", ls=":", label="FOC flux request")
axes[3].step(time, values(rows, "brake_nm"), color="#555555", ls="--", where="post", label="brake capacity")
axes[3].legend(loc="upper left", fontsize=9)
for ax, label in zip(axes, ["Speed (rad/s)", "Current vector (A)", "Rotor flux (Wb turns)", "Torque (Nm)"]):
    ax.set_ylabel(label)
    ax.axvspan(5.5, 7.5, color="#d23d45", alpha=.08)
    ax.set_xlim(0, 11)
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Ideal rotor-flux FOC regulates speed and flux; V/Hz does not\n"
             "Same induction plant and voltage ceiling; 6 Nm load, 20 Nm overload, then recovery", fontsize=13)
fig.supxlabel("FOC: true rotor-flux feedback, 8 A vector current-reference limit. V/Hz: no current protection.\n"
              "Current shown is a vector magnitude, not transient phase RMS; comparison is not equal-current constrained.", fontsize=9)
save(fig, "induction_foc_comparison")

fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
for name, color in colors.items():
    rows = [r for r in groups[name] if 3.35 <= r["time_s"] <= 3.9]
    time = values(rows, "time_s")
    axes[0].plot(time, values(rows, "rotor_rad_s"), color=color, label=name)
    axes[2].plot(time, values(rows, "voltage_vector_v"), color=color)
rows = [r for r in groups["ideal flux FOC"] if 3.35 <= r["time_s"] <= 3.9]
for key, reference, label, color in [("id_a", "id_reference_a", "d / flux current", "#2675bf"),
                                     ("iq_a", "iq_reference_a", "q / torque current", "#9049a9")]:
    axes[1].plot(time, values(rows, key), color=color, label=label)
    axes[1].plot(time, values(rows, reference), color=color, ls="--", alpha=.7)
axes[0].axhline(150, color="#555555", ls=":")
axes[2].axhline(voltage_limit, color="#555555", ls=":", label="voltage ceiling")
axes[0].legend(loc="lower left")
axes[1].legend(loc="lower right")
axes[2].legend(loc="lower right")
for ax, label in zip(axes, ["Speed (rad/s)", "FOC dq current (A)", "Voltage vector (V)"]):
    ax.set_ylabel(label)
    ax.axvline(3.5, color="gray", ls=":")
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Load rejection is finite-bandwidth motion, not an assigned rotor speed\n"
             "6 Nm applied at 3.5 s; FOC current references dashed, actual currents solid", fontsize=13)
save(fig, "induction_foc_load_rejection")

rows = groups["FOC voltage ceiling stress"]
time = values(rows, "time_s")
fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
axes[0].plot(time, values(rows, "speed_reference_rad_s"), color="#555555", ls="--", label="request")
axes[0].plot(time, values(rows, "rotor_rad_s"), color="#2675bf", label="actual rotor")
axes[0].legend(loc="upper right")
axes[1].plot(time, values(rows, "current_vector_a"), color="#2675bf", label="actual stator current")
axes[1].plot(time, np.hypot(values(rows, "id_reference_a"), values(rows, "iq_reference_a")),
             color="#9049a9", ls="--", label="vector current reference")
axes[1].axhline(8, color="#555555", ls=":")
axes[1].legend(loc="upper right")
axes[2].plot(time, values(rows, "voltage_vector_v"), color="#2675bf")
axes[2].axhline(voltage_limit, color="#555555", ls="--")
axes[3].plot(time, values(rows, "rotor_flux_wb"), color="#2675bf")
axes[3].axhline(.5, color="#555555", ls=":")
for ax, label in zip(axes, ["Speed (rad/s)", "Current vector (A)", "Voltage vector (V)", "Rotor flux (Wb turns)"]):
    ax.set_ylabel(label)
    ax.fill_between(time, 0, 1, where=values(rows, "voltage_limited") != 0,
                    transform=ax.get_xaxis_transform(), color="#d23d45", alpha=.07)
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Anti-windup permits recovery; it does not remove the voltage ceiling\n"
             "Unattainable 350 rad/s request, then 100 rad/s at 3.5 s; true-flux FOC", fontsize=13)
fig.supxlabel("Shading: sampled voltage-saturation flag. Actual current can exceed its reference limit:\n"
              "finite current-loop bandwidth and voltage clipping are not hardware overcurrent protection.", fontsize=9)
save(fig, "induction_foc_voltage_limit")

summary = source.with_name(source.stem + "_summary.csv")
with summary.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["drive", "window_start_s", "window_end_s", "mean_speed_rad_s", "mean_current_vector_a",
                     "mean_rotor_flux_wb", "mean_torque_nm", "mean_voltage_vector_v"])
    for name, rows in groups.items():
        windows = [(3.3, 3.5), (5.8, 6)] if "stress" in name else [(3.3, 3.5), (5.3, 5.5), (7.3, 7.5), (10.8, 11)]
        for start, end in windows:
            selected = [r for r in rows if start < r["time_s"] <= end]
            means = [values(selected, key).mean() for key in
                     ["rotor_rad_s", "current_vector_a", "rotor_flux_wb", "torque_nm", "voltage_vector_v"]]
            writer.writerow([name, start, end, *means])
print(f"Wrote FOC comparison, load rejection and voltage-limit figures to {out}; summary: {summary}")
