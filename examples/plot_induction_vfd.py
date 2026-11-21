#!/usr/bin/env python3
"""Plot averaged induction/VFD experiments. NumPy/Matplotlib are optional.
Usage: python3 examples/plot_induction_vfd.py [CSV [OUTPUT_DIR]]
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/induction_vfd.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row.pop("scenario"), []).append({k: float(v) for k, v in row.items()})
plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .23})


def array(rows, key):
    return np.array([r[key] for r in rows])


def save(fig, name):
    for ext in ("png", "svg"):
        fig.savefig(out / f"{name}.{ext}", dpi=160)
    plt.close(fig)


rows = groups["ramped 50 Hz"]
time = array(rows, "time_s")
fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
axes[0].plot(time, array(rows, "synchronous_rad_s")*60/(2*np.pi), color="#555555", ls="--", label="rotating field")
axes[0].plot(time, array(rows, "rotor_rad_s")*60/(2*np.pi), color="#2675bf", label="rotor")
axes[0].set_ylabel("Speed (rpm)")
axes[0].legend(loc="lower right")
axes[1].plot(time, array(rows, "slip_hz"), color="#9049a9")
axes[1].set_ylabel("Slip frequency (Hz)")
axes[2].plot(time, array(rows, "torque_nm"), color="#2675bf", label="electromagnetic torque")
axes[2].step(time, array(rows, "brake_nm"), color="#d08c13", where="post", ls="--", label="brake capacity")
axes[2].set_ylabel("Torque (Nm)")
axes[2].legend(loc="upper right")
axes[3].plot(time, array(rows, "stator_current_vector_a"), color="#2675bf")
axes[3].set(ylabel="Stator current vector (A)", xlabel="Simulated time (s)")
for ax in axes:
    ax.axvspan(5, 8, color="#d23d45", alpha=.08)
    ax.set_xlim(0, 11)
fig.suptitle("Open-loop V/Hz: load causes slip, overload causes stall\n"
             "Four-pole induction motor; 25 Hz/s ramp to 50 Hz; no rotor feedback to drive", fontsize=13)
fig.supxlabel("Brake is passive: its 20 Nm stalled holding capacity is not the torque actually absorbed at zero speed.\n"
              "No current protection, saturation or thermal model; illustrative parameters, not a motor rating.", fontsize=9)
save(fig, "induction_vfd_load")

fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, constrained_layout=True)
for name, color in [("ramped 50 Hz", "#2675bf"), ("direct 50 Hz", "#d08c13")]:
    rows = [r for r in groups[name] if r["time_s"] <= 2.5]
    time = array(rows, "time_s")
    axes[0].plot(time, array(rows, "rotor_rad_s")*60/(2*np.pi), label=name, color=color)
    axes[0].plot(time, array(rows, "synchronous_rad_s")*60/(2*np.pi), ls=":", color=color, alpha=.6)
    axes[1].plot(time, array(rows, "stator_current_vector_a"), color=color)
    axes[2].plot(time, array(rows, "rotor_flux_wb"), color=color)
axes[0].legend(loc="lower right")
axes[0].set_ylabel("Speed (rpm)")
axes[1].set_ylabel("Stator current vector (A)")
axes[2].set(ylabel="Rotor flux linkage (Wb turns)", xlabel="Simulated time (s)")
fig.suptitle("Abrupt 50 Hz application versus a V/Hz frequency ramp\n"
             "Dotted curves: field speed; solid curves: rotor speed; unloaded startup", fontsize=13)
fig.supxlabel("Current is instantaneous alpha/beta vector magnitude, not transient phase RMS.\n"
              "Voltage is limited, but no current-limit/trip logic is implemented.", fontsize=9)
save(fig, "induction_vfd_startup")

fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
for name, color in [("5 Hz no boost", "#d08c13"), ("5 Hz boost", "#2675bf")]:
    rows = groups[name]
    time = array(rows, "time_s")
    axes[0].plot(time, array(rows, "rotor_rad_s")*60/(2*np.pi), color=color, label=name)
    axes[1].plot(time, array(rows, "torque_nm"), color=color)
    axes[2].plot(time, array(rows, "stator_current_vector_a"), color=color)
    axes[3].plot(time, array(rows, "rotor_flux_wb"), color=color)
axes[0].plot(time, array(rows, "synchronous_rad_s")*60/(2*np.pi), color="#555555", ls=":", label="field speed")
axes[0].legend(loc="upper right")
axes[0].set_ylabel("Speed (rpm)")
axes[1].step(time, array(rows, "brake_nm"), color="#555555", ls="--", where="post")
axes[1].set_ylabel("Torque / brake (Nm)")
axes[2].set_ylabel("Stator current vector (A)")
axes[3].set(ylabel="Rotor flux linkage (Wb turns)", xlabel="Simulated time (s)")
for ax in axes:
    ax.axvline(1, color="gray", ls=":", alpha=.6)
fig.suptitle("Low-frequency voltage boost can restore torque margin\n"
             "5 Hz drive, 3 Nm brake applied at 1 s; boost intercept = 10 phase-peak volts", fontsize=13)
fig.supxlabel("Boost also increases flux. The linear model omits saturation and heating:\n"
              "this is a mechanism demonstration, not a recommended boost setting.", fontsize=9)
save(fig, "induction_vfd_low_speed")
# Preserve compact steady-window numbers alongside generated source data.
summary = source.with_name(source.stem + "_summary.csv")
with summary.open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["scenario", "window_start_s", "window_end_s", "mean_speed_rpm",
                     "mean_slip_hz", "mean_torque_nm", "mean_current_vector_a", "mean_rotor_flux_wb"])
    for name, rows in groups.items():
        windows = [(2.8, 3), (4.8, 5), (7.8, 8), (10.8, 11)] if "50 Hz" in name else [(3.8, 4)]
        for start, end in windows:
            selected = [r for r in rows if start < r["time_s"] <= end]
            values = [array(selected, k).mean() for k in
                      ["rotor_rad_s", "slip_hz", "torque_nm", "stator_current_vector_a", "rotor_flux_wb"]]
            values[0] *= 60/(2*np.pi)
            writer.writerow([name, start, end, *values])
print(f"Wrote induction load, startup and low-speed figures to {out}; summary: {summary}")
