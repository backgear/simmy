#!/usr/bin/env python3
"""Plot a single healthy BLDC run, as a baseline before the fault/heat plots.
Usage: python3 examples/plot_bldc_healthy.py [CSV [OUTPUT_DIR]]
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/bldc_healthy.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
with source.open(newline="") as f:
    rows = [row for row in csv.DictReader(f) if row["command_sample"] == "1"]
time = np.array([float(r["time_s"]) for r in rows])
reference = np.array([float(r["reference_rad_s"]) for r in rows])
speed = np.array([float(r["speed_rad_s"]) for r in rows])
torque = np.array([float(r["torque_nm"]) for r in rows])
brake = np.array([float(r["brake_nm"]) for r in rows])
phase_a = np.array([float(r["iwa_a"]) for r in rows])
phase_b = np.array([float(r["iwb_a"]) for r in rows])
phase_c = np.array([float(r["iwc_a"]) for r in rows])

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
axes[0].plot(time, reference, color="#555555", ls="--", label="reference")
axes[0].plot(time, speed, color="#2675bf", label="actual")
axes[0].set_ylabel("Shaft speed (rad/s)")
axes[0].legend(loc="lower right")
axes[1].plot(time, torque, color="#20a080", label="electromagnetic torque")
axes[1].step(time, brake, color="#d08c13", where="post", ls="--", label="brake capacity")
axes[1].set_ylabel("Torque (Nm)")
axes[1].legend(loc="upper right")
axes[2].plot(time, phase_a, label="phase A", lw=.8)
axes[2].plot(time, phase_b, label="phase B", lw=.8)
axes[2].plot(time, phase_c, label="phase C", lw=.8)
axes[2].set(ylabel="Winding current (A)", xlabel="Simulated time (s)")
axes[2].legend(loc="upper right", ncol=3)
for ax in axes:
    ax.axvspan(.5, 1.5, color="#d23d45", alpha=.06)
fig.suptitle("Healthy BLDC baseline: 1 kHz speed PI, 20 kHz six-step, 5 kHz PWM\n"
             "Four-pole wye motor, trapezoidal back-EMF; 0.12 Nm brake 0.5-1.5s (shaded)",
             fontsize=13)
fig.supxlabel("This is the case every fault/heat comparison on the BLDC page is measured against.",
              fontsize=9)
for extension in ("png", "svg"):
    fig.savefig(out / f"bldc_healthy.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {out}/bldc_healthy.{{png,svg}}")
