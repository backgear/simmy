#!/usr/bin/env python3
"""Optional plotting only; library/tests still require just C + libm.
Usage: python3 examples/plot_solenoid_actuators.py [CSV [OUTPUT_DIR]]
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/solenoid_actuators.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
with source.open(newline="") as f:
    rows = list(csv.DictReader(f))


def column(name, rows):
    return np.array([float(r[name]) for r in rows])


reluctance = [r for r in rows if r["family"] == "reluctance_pullin"]
voicecoil = [r for r in rows if r["family"] == "voicecoil_pushpull"]

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(3, 2, figsize=(13, 9), sharex="col", constrained_layout=True)

t = column("time", reluctance)
axes[0, 0].plot(t, column("current", reluctance) * 1e3, color="#2675bf")
axes[0, 0].set_ylabel("Current (mA)")
axes[1, 0].plot(t, column("position", reluctance) * 1e3, color="#d08c13")
axes[1, 0].axhline(.2, color="gray", ls="--", alpha=.6)
axes[1, 0].annotate("electrical gap bound (0.2mm)", xy=(t[-1], .2), xytext=(t[-1] * .55, .5),
                     fontsize=8, color="gray")
axes[1, 0].set_ylabel("Air gap (mm)")
axes[2, 0].plot(t, column("velocity", reluctance) * 1e3, color="#20a080")
axes[2, 0].set(ylabel="Gap velocity (mm/s)", xlabel="Simulated time (s)")
axes[0, 0].set_title("Reluctance (iron-core): constant 6V -- pull-in ONLY,\nnever repulsive")

t2 = column("time", voicecoil)
axes[0, 1].plot(t2, column("voltage", voicecoil), color="#9b3fd4", label="voltage (V)")
axes[0, 1].plot(t2, column("current", voicecoil), color="#2675bf", ls="--", label="current (A)")
axes[0, 1].legend(fontsize=8, loc="upper right")
axes[0, 1].set_ylabel("Voltage / current")
axes[1, 1].plot(t2, column("position", voicecoil) * 1e3, color="#d08c13")
axes[1, 1].set_ylabel("Position (mm)")
axes[2, 1].plot(t2, column("velocity", voicecoil) * 1e3, color="#20a080")
axes[2, 1].axhline(0, color="gray", lw=.8)
axes[2, 1].set(ylabel="Velocity (mm/s)", xlabel="Simulated time (s)")
axes[0, 1].set_title("Voice coil: 0.5Hz/8V sine -- PUSH and PULL,\nforce follows signed current")

fig.suptitle("Linear electromagnetic actuators: attraction-only reluctance vs. bidirectional voice coil",
             fontsize=13)
for extension in ("png", "svg"):
    fig.savefig(out / f"solenoid_actuators.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {out}/solenoid_actuators.{{png,svg}}")
