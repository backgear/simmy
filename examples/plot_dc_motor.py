#!/usr/bin/env python3
"""Optional plotting only; library/tests still require just C + libm.
Usage: python3 examples/plot_dc_motor.py [CSV [OUTPUT_DIR]]
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/dc_motor.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
with source.open(newline="") as f:
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
time = np.array([r["time"] for r in rows])
speed = np.array([r["speed"] for r in rows])
current = np.array([r["current"] for r in rows])
torque = np.array([r["torque"] for r in rows])
brake_time = 1.0

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
axes[0].plot(time, speed, color="#2675bf")
axes[0].set_ylabel("Speed (rad/s)")
axes[1].plot(time, current, color="#d08c13")
axes[1].set_ylabel("Current (A)")
axes[2].plot(time, torque, color="#20a080")
axes[2].set(ylabel="Torque (Nm)", xlabel="Simulated time (s)")
for ax in axes:
    ax.axvline(brake_time, color="gray", ls="--", alpha=.6)
axes[0].annotate("0.05 Nm brake engages", xy=(brake_time, speed.max()),
                  xytext=(brake_time + .05, speed.max()), fontsize=9, color="gray")
fig.suptitle("Brushed DC motor: constant 12V, no speed controller\n"
             "R=0.6 ohm, L=0.002 H, Ke=Kt=0.08 Nm/A; single body, J=8e-4 kg m^2, B=5e-4 Nm/(rad/s)",
             fontsize=13)
fig.supxlabel("No commutation and no drive clock: the electrical step runs on the mechanical tick.\n"
              "Current/torque step instantly at t=0 and at the brake because there is no rotor-angle "
              "dependence to delay them.", fontsize=9)
for extension in ("png", "svg"):
    fig.savefig(out / f"dc_motor.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {out}/dc_motor.{{png,svg}}")
