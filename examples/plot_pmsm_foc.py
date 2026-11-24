#!/usr/bin/env python3
"""Plot continuous-angle (resolver-style) PMSM FOC tracking, load rejection
and a separate voltage-limit stress/recovery run.
Usage: python3 examples/plot_pmsm_foc.py [CSV [OUTPUT_DIR]]
NumPy/Matplotlib are plotting-only dependencies.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/pmsm_foc.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row.pop("drive"), []).append({k: float(v) for k, v in row.items()})
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})
voltage_limit = 24/np.sqrt(3)
current_limit = 8


def values(rows, key):
    return np.array([r[key] for r in rows])


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(out / f"{name}.{ext}", dpi=160)
    plt.close(fig)


rows = groups["PMSM FOC"]
time = values(rows, "time_s")
fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
axes[0].plot(time, values(rows, "speed_reference_rad_s"), color="#555555", ls="--", label="speed request")
axes[0].plot(time, values(rows, "speed_rad_s"), color="#2675bf", label="actual speed")
axes[0].legend(loc="lower right", fontsize=9)
axes[1].plot(time, values(rows, "iq_a"), color="#9049a9", label="measured Iq")
axes[1].plot(time, values(rows, "iq_reference_a"), color="#9049a9", ls="--", alpha=.7, label="Iq reference")
axes[1].plot(time, values(rows, "id_a"), color="#2675bf", label="measured Id (reference 0)")
axes[1].legend(loc="upper left", fontsize=9)
axes[2].plot(time, values(rows, "current_vector_a"), color="#2675bf", label="stator current vector")
axes[2].axhline(current_limit, color="#555555", ls=":", label="current-reference limit")
axes[2].legend(loc="upper left", fontsize=9)
axes[3].plot(time, values(rows, "voltage_vector_v"), color="#2675bf")
axes[3].axhline(voltage_limit, color="#555555", ls=":", label="voltage ceiling")
axes[3].step(time, values(rows, "brake_nm")*10, color="#d23d45", ls="--", where="post", label="brake (x10 Nm, for scale)")
axes[3].legend(loc="upper left", fontsize=9)
for ax, label in zip(axes, ["Speed (rad/s)", "dq current (A)", "Current vector (A)", "Voltage vector (V)"]):
    ax.set_ylabel(label)
    ax.set_xlim(0, time[-1])
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Continuous sensed-angle (resolver-style) FOC on simmy's sine-PM plant\n"
             "0.3 Nm brake applied 1.5-3.5 s; Id held at 0 (MTPA, non-salient model)", fontsize=13)
fig.supxlabel("Reference harness for testing external no-Hall FOC-commutated servo-amp firmware\n"
              "(e.g. an AMC SX25A-class drive) -- simmy has no Hall-table commutation model.", fontsize=9)
save(fig, "pmsm_foc_tracking")

rows = groups["PMSM FOC voltage ceiling stress"]
time = values(rows, "time_s")
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
axes[0].plot(time, values(rows, "speed_reference_rad_s"), color="#555555", ls="--", label="request")
axes[0].plot(time, values(rows, "speed_rad_s"), color="#2675bf", label="actual speed")
axes[0].legend(loc="upper right", fontsize=9)
axes[1].plot(time, values(rows, "current_vector_a"), color="#2675bf", label="actual stator current")
axes[1].plot(time, np.hypot(values(rows, "id_reference_a"), values(rows, "iq_reference_a")),
             color="#9049a9", ls="--", label="vector current reference")
axes[1].axhline(current_limit, color="#555555", ls=":")
axes[1].legend(loc="upper right", fontsize=9)
axes[2].plot(time, values(rows, "voltage_vector_v"), color="#2675bf")
axes[2].axhline(voltage_limit, color="#555555", ls="--")
for ax, label in zip(axes, ["Speed (rad/s)", "Current vector (A)", "Voltage vector (V)"]):
    ax.set_ylabel(label)
    ax.fill_between(time, 0, 1, where=values(rows, "voltage_limited") != 0,
                    transform=ax.get_xaxis_transform(), color="#d23d45", alpha=.07)
axes[-1].set_xlabel("Simulated time (s)")
fig.suptitle("Anti-windup permits recovery; it does not remove the voltage ceiling\n"
             "Unattainable 300 rad/s request, then 100 rad/s at 2 s", fontsize=13)
fig.supxlabel("Shading: sampled voltage-saturation flag. Actual current can exceed its reference limit:\n"
              "finite current-loop bandwidth and voltage clipping are not hardware overcurrent protection.", fontsize=9)
save(fig, "pmsm_foc_voltage_limit")

print(f"Wrote PMSM FOC tracking and voltage-limit figures to {out}")
