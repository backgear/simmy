#!/usr/bin/env python3
"""Plot independent speed trials; requires NumPy + Matplotlib.
Usage: python3 examples/plot_stepper_sweep.py [CSV [OUTPUT_PREFIX]]
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/stepper_sweep.csv")
prefix = Path(sys.argv[2] if len(sys.argv) > 2 else "build/stepper_sweep")
prefix.parent.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row.pop("scenario"), []).append({k: float(v) for k, v in row.items()})
colors = ["#555555", "#2675bf", "#20a080", "#d08c13", "#9049a9", "#d23d45"]
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True, constrained_layout=True)
print("Peak normalized RMS per scenario (sampled speeds, not optimized maxima):")
print(f"{'scenario':24} {'rad/s':>10} {'rpm':>10} {'RMS %':>12} {'RMS rad/s':>12} {'crossings*':>12}")
all_speeds = []
for (name, rows), color in zip(groups.items(), colors):
    rows.sort(key=lambda r: r["target_rad_s"])
    speed = np.array([r["target_rad_s"] for r in rows])
    rms = np.array([r["rms_error_rad_s"] for r in rows])
    crossed = np.array([r["phase_boundary_crossed"] != 0 for r in rows])
    relative = 100*rms/speed
    axes[0].loglog(speed, rms, color=color, label=name, lw=1.4)
    axes[1].loglog(speed, relative, color=color, lw=1.4)
    axes[2].semilogx(speed, [100*r["relative_mean_error"] for r in rows], color=color, lw=1.2)
    axes[3].semilogx(speed, [100*r["reverse_fraction"] for r in rows], color=color, lw=1.2)
    if crossed.any():
        axes[1].scatter(speed[crossed], relative[crossed], marker="x", s=30, color=color)
    i = int(np.argmax(relative))
    print(f"{name:24} {speed[i]:10.6f} {speed[i]*60/(2*np.pi):10.4f} "
          f"{relative[i]:12.3f} {rms[i]:12.5f} {int(crossed.sum()):12d}")
    all_speeds.extend(speed)
print("* Number of runs crossing +/- pi unwrapped electrical load angle, including settling.")
print("Crossing is a phase-well warning, not proof of permanent loss of synchronism.")
print("x markers denote those warnings; independent equilibrium starts, not an acceleration ramp.")

# Local small-signal resonance about ideal loaded traveling equilibrium.
# Damping/nonlinearity and harmonic excitation can shift or add peaks.
load_angle = np.arcsin((.03+.008*.2)/.4)
f_n = np.sqrt(.4*50*np.cos(load_angle)/4e-4)/(2*np.pi)
markers = [(2*np.pi*f_n/(200*64), "64 µstep fundamental"),
           (2*np.pi*f_n/(200*16), "16 µstep fundamental"),
           (2*np.pi*f_n/200, "detent fundamental")]
for speed, label in markers:
    for ax in axes:
        ax.axvline(speed, color="gray", alpha=.5, ls="--", lw=.9)
    axes[0].text(speed, .98, f"{label}\n{speed:.4f} rad/s", ha="center", va="top",
                 transform=axes[0].get_xaxis_transform(), fontsize=8,
                 bbox={"facecolor": "white", "alpha": .8, "edgecolor": "none"})
axes[0].set_ylabel("RMS error (rad/s)")
axes[0].legend(loc="lower right", ncol=3, fontsize=9)
axes[1].set_ylabel("RMS error / target (%)")
axes[2].set_ylabel("Mean-speed error (%)")
axes[3].set(ylabel="Time moving backward (%)", xlabel="Commanded mechanical speed (rad/s)", ylim=(-2, 55))
axes[3].set_xlim(min(all_speeds), max(all_speeds))
axes[0].secondary_xaxis("top", functions=(lambda x: x*60/(2*np.pi), lambda x: x*2*np.pi/60)).set_xlabel("Speed (rpm)")
fig.suptitle("Stepper ripple vs. speed — illustrative current-driven motor, no velocity feedback\n"
             f"Dashed lines: excitation fundamental meets local resonance ≈ {f_n:.1f} Hz; "
             "higher harmonics can excite it too", fontsize=13)
fig.supxlabel("Pulse times are outer-tick quantized; sub-percent high-speed tails remain timestep-sensitive.",
              fontsize=9)
for ext in ("png", "svg"):
    fig.savefig(f"{prefix}.{ext}", dpi=160)
plt.close(fig)
print(f"Wrote {prefix}.{{png,svg}}")
