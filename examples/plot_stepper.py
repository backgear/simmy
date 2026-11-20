#!/usr/bin/env python3
"""Optional plotting only; library/tests still require just C + libm.
Usage: python3 examples/plot_stepper.py build/stepper_velocity.csv build/stepper
Requires numpy and matplotlib. Outputs PNG/SVG and prints steady-window metrics.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/stepper_velocity.csv")
prefix = Path(sys.argv[2] if len(sys.argv) > 2 else "build/stepper")
prefix.parent.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row["scenario"], []).append(
            [float(row[k]) for k in ("time_s", "position_rad", "velocity_rad_s", "torque_nm")])
groups = {key: np.asarray(rows) for key, rows in groups.items()}
colors = dict(zip(groups, ["#555555", "#2675bf", "#20a080", "#d08c13", "#9049a9", "#d23d45"]))
target = .2
print("Steady window: 2–4 s, target 0.2 rad/s; illustrative parameters")
print(f"{'scenario':24} {'mean rad/s':>12} {'RMS error %':>13} {'p-p %':>10}")
for name, data in groups.items():
    v = data[:, 2]
    print(f"{name:24} {v.mean():12.7f} {100*np.sqrt(np.mean((v-target)**2))/target:13.4f} "
          f"{100*np.ptp(v)/target:10.4f}")

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True, constrained_layout=True)
comparisons = [
    ("Command discretization alone", ["continuous ideal", "16 microsteps", "64 microsteps"]),
    ("Motor / current-path disturbances with continuous commands", ["continuous ideal", "detent only", "current errors only"]),
    ("Disturbances combined with discrete microsteps", ["16 microsteps", "combined 16"]),
]
for ax, (title, names) in zip(axes, comparisons):
    for name in names:
        data = groups[name]
        use = (data[:, 0] >= 3) & (data[:, 0] <= 3.6)
        ax.plot(data[use, 0], 100*(data[use, 2]/target-1), color=colors[name], label=name, lw=1.1)
    ax.set(title=title, ylabel="Velocity error (%)")
    ax.legend(loc="upper right", ncol=len(names))
axes[-1].set_xlabel("Time (s)")
fig.suptitle("Stepper velocity ripple — 0.2 rad/s (1.91 rpm), 1 A vector current\n"
             "Illustrative motor: 200 full steps/rev, Kt=0.4 Nm/A, J=4e−4 kg m²; no velocity feedback")
for extension in ("png", "svg"):
    fig.savefig(f"{prefix}_velocity.{extension}", dpi=160)
plt.close(fig)

fig, ax = plt.subplots(figsize=(11, 5), constrained_layout=True)
for name, data in groups.items():
    if name == "continuous ideal":
        continue
    v = data[:, 2]
    window = np.hanning(len(v))
    amplitude = 2*np.abs(np.fft.rfft((v-v.mean())*window))/window.sum()
    frequency = np.fft.rfftfreq(len(v), d=float(np.median(np.diff(data[:, 0]))))
    ax.semilogy(frequency, np.maximum(amplitude/target*100, 1e-8), label=name, color=colors[name], lw=1)
for freq, label, text_x, text_y in [
    (50*target/(2*np.pi), "electrical", 15, .94),
    (200*target/(2*np.pi), "detent", 55, .80),
    (50*target*32/np.pi, "16 µstep updates", 120, .94),
]:
    ax.axvline(freq, color="gray", alpha=.4, ls="--")
    ax.annotate(f"{label}: {freq:.2f} Hz", xy=(freq, text_y),
                xytext=(text_x, text_y), xycoords=ax.get_xaxis_transform(),
                textcoords=ax.get_xaxis_transform(), fontsize=8,
                arrowprops={"arrowstyle": "-", "color": "gray"})
ax.set(xlim=(0, 450), ylim=(1e-4, 100), xlabel="Frequency (Hz)",
       ylabel="Velocity amplitude (% of target)",
       title="Ripple spectrum — 2–4 s window, mean removed, Hann-windowed\n"
             "Finite-window amplitudes are approximate; not a measured motor spectrum")
ax.legend(loc="upper right")
for extension in ("png", "svg"):
    fig.savefig(f"{prefix}_spectrum.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {prefix}_{{velocity,spectrum}}.{{png,svg}}")
