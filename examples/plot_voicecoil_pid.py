#!/usr/bin/env python3
"""Optional plotting only; library/tests still require just C + libm.
Usage: python3 examples/plot_voicecoil_pid.py [CSV [OUTPUT_DIR]]
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/voicecoil_pid.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
with source.open(newline="") as f:
    rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
time = np.array([r["time"] for r in rows])
target = np.array([r["target"] for r in rows])
position = np.array([r["position"] for r in rows])
velocity = np.array([r["velocity"] for r in rows])
voltage = np.array([r["voltage"] for r in rows])
current = np.array([r["current"] for r in rows])
step_times = (.5, 2.5)

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
axes[0].plot(time, target, color="gray", ls="--", label="target")
axes[0].plot(time, position, color="#2675bf", label="position")
axes[0].set_ylabel("Position (counts)")
axes[0].legend(fontsize=8, loc="center right")
axes[1].plot(time, velocity, color="#20a080")
axes[1].axhline(0, color="gray", lw=.8)
axes[1].set_ylabel("Velocity (counts/tick)")
axes[2].plot(time, voltage, color="#9b3fd4")
axes[2].set_ylabel("Voltage (V)")
axes[3].plot(time, current, color="#d08c13")
axes[3].set(ylabel="Current (A)", xlabel="Simulated time (s)")
for ax in axes:
    for st in step_times:
        ax.axvline(st, color="gray", ls=":", alpha=.6)
axes[0].annotate("lift command", xy=(step_times[0], target.max()),
                  xytext=(step_times[0] + .05, target.max() * .85), fontsize=8, color="gray")
axes[0].annotate("descend command", xy=(step_times[1], target.max()),
                  xytext=(step_times[1] + .05, target.max() * .85), fontsize=8, color="gray")
fig.suptitle("Vertical voice-coil actuator: closed-loop PID position control against gravity\n"
             "No mechanical return spring -- gravityBias alone pulls down; PID must both lift AND brake the descent",
             fontsize=12)
fig.supxlabel("Voltage briefly saturates at each step (PID output clamped); current settles near the "
              "gravityBias/forceConstant hold value between moves, not drifting with a P-only steady-state sag.",
              fontsize=9)
for extension in ("png", "svg"):
    fig.savefig(out / f"voicecoil_pid.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {out}/voicecoil_pid.{{png,svg}}")
