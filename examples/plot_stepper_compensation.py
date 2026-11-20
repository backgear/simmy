#!/usr/bin/env python3
"""Optional blog figures; requires numpy/matplotlib.
Usage: python3 examples/plot_stepper_compensation.py [BUILD_DIR [OUTPUT_DIR]]
Reads the sweep and three traces documented in docs/stepper-blog-notes.md.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

build = Path(sys.argv[1] if len(sys.argv) > 1 else "build")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
policies = ["none", "detent feedforward", "active damping", "both"]
colors = ["#555555", "#d08c13", "#2675bf", "#b53b82"]
styles = ["-", "--", "-", "--"]
scenarios = ["detent only", "16 microsteps", "combined 16"]
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(out / f"{name}.{ext}", dpi=160)
    plt.close(fig)


with (build / "stepper_compensation_sweep.csv").open(newline="") as f:
    rows = list(csv.DictReader(f))
fig, axes = plt.subplots(3, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
for ax, scenario in zip(axes, scenarios):
    for policy, color, style in zip(policies, colors, styles):
        data = [r for r in rows if r["scenario"] == scenario and r["policy"] == policy]
        data.sort(key=lambda r: float(r["target_rad_s"]))
        speed = np.array([float(r["target_rad_s"]) for r in data])
        error = np.array([float(r["rms_error_rad_s"]) for r in data])
        ax.loglog(speed, 100*error/speed, label=policy, color=color, ls=style, lw=1.5)
    ax.set(title=scenario, ylabel="RMS velocity error / target (%)", xlim=(.01, 1.2))
axes[0].legend(ncol=2, loc="lower left")
axes[-1].set_xlabel("Mechanical target speed (rad/s)")
fig.suptitle("Compensation addresses different causes of velocity ripple\n"
             "Same motor and drive errors; ideal rotor feedback; no command smoothing", fontsize=13)
fig.supxlabel("Exact detent estimate; damping = 0.08 Nm/(rad/s); correction passes through phase-current paths.\n"
              "Microstep-only ablation has no detent/table. Small high-speed residuals are timestep-sensitive.", fontsize=9)
save(fig, "stepper_compensation_sweep")

fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True)
for ax, key, speed, title in zip(axes, ["detent", "microsteps", "combined"], [1.107971, .0698, .2],
                               ["Detent resonance", "16-microstep resonance", "Combined disturbances"]):
    with (build / f"stepper_compensation_{key}_trace.csv").open(newline="") as f:
        trace = list(csv.DictReader(f))
    for policy, color, style in zip(policies, colors, styles):
        data = [r for r in trace if r["policy"] == policy and 3 <= float(r["time_s"]) <= 3.4]
        ax.plot([float(r["time_s"])-3 for r in data],
                [float(r["velocity_rad_s"]) for r in data],
                label=policy, color=color, ls=style, lw=1.2)
    ax.axhline(speed, color="black", lw=.8, alpha=.5, label="target" if key == "detent" else None)
    ax.set(title=f"{title} — target {speed:g} rad/s", ylabel="Rotor velocity (rad/s)", xlabel="Time within excerpt (s)")
axes[0].legend(ncol=3, loc="upper right", fontsize=9)
fig.suptitle("Average speed is not motion quality\n"
             "Steady excerpts after 3 seconds; negative velocity means real reversal, not a plotting artifact", fontsize=13)
save(fig, "stepper_compensation_traces")

# A conceptual figure: magnetic field is an excitation, not a rigid body.
fig, ax = plt.subplots(figsize=(12, 4.5), constrained_layout=True)
ax.set(xlim=(0, 12), ylim=(0, 4.5))
ax.axis("off")
boxes = [(1.1, "Command\nfield angle φ"), (4.1, "Phase-current paths\niA, iB"),
         (7.2, "Magnetic torque\nf(iA, iB, Nθ)\n+ detent"),
         (10.5, "Mechanical rotor\ninertia + friction\nθ, ω → encoder")]
for x, label in boxes:
    ax.text(x, 2.8, label, ha="center", va="center", fontsize=11,
            bbox={"boxstyle": "round,pad=.6", "facecolor": "#edf2f7", "edgecolor": "#53677b"})
for start, end in [(2.05, 2.95), (5.35, 5.95), (8.5, 9.2)]:
    ax.annotate("", xy=(end, 2.8), xytext=(start, 2.8), arrowprops={"arrowstyle": "->", "lw": 1.6})
ax.plot([10.5, 10.5, 7.2], [2.1, 1.4, 1.4], color="#53677b", lw=1.2)
ax.annotate("", xy=(7.2, 2.1), xytext=(7.2, 1.4),
            arrowprops={"arrowstyle": "->", "color": "#53677b"})
ax.text(8.8, 1, "Actual rotor angle determines torque", ha="center", color="#53677b")
ax.text(4.5, .35, "Optional compensation: encoder θ, ω → current correction\n"
                  "The command never assigns rotor position directly.", ha="center", fontsize=11)
ax.set_title("Electromechanical separation is already in the model", fontsize=15)
save(fig, "stepper_electromechanical_separation")
print(f"Wrote compensation sweep, traces, and electromechanical diagram to {out}")
