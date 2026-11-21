#!/usr/bin/env python3
"""BLDC fault heat, winding/terminal currents and reduced-model validation.
Optional plotting only: NumPy/Matplotlib. Scoring helpers use stdlib alone.
"""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from bldc_metrics import window_stats, normalize

VARIANTS = ["healthy", "advance30", "wrong_sector", "high_r", "high_ron", "open_high", "open_winding", "reverse_sector"]
LABELS = ["Healthy", "+30° timing", "+60° wrong sector", "Winding A/AB R ×5", "A high FET R ↑", "A high FET open", "Winding A/AB open", "+120° wrong sector"]


def load(path, waveforms=False):
    groups, phases = {}, {}
    with path.open(newline="") as f:
        for row in csv.DictReader(f):
            name = row["case"]
            if int(row["command_sample"]):
                groups.setdefault(name, []).append({k: float(v) for k, v in row.items() if k != "case"})
            if waveforms and 1.25 < float(row["time_s"]) <= 1.29:
                if name not in phases:
                    phases[name] = dict(count=np.zeros(180), total=np.zeros((2, 180)),
                                        low=np.full((2, 180), np.inf), high=np.full((2, 180), -np.inf))
                p = phases[name]
                index = int((4*float(row["angle_rad"]) % (2*np.pi))*180/(2*np.pi))
                p["count"][index] += 1
                for j, key in enumerate(["iwa_a", "ila_a"]):
                    value = float(row[key])
                    p["total"][j, index] += value
                    p["low"][j, index] = min(p["low"][j, index], value)
                    p["high"][j, index] = max(p["high"][j, index], value)
    return groups, phases


def save(fig, out, name):
    for ext in ("png", "svg"):
        fig.savefig(out/f"{name}.{ext}", dpi=160)
    plt.close(fig)


def csv_write(path, rows):
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]), lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def values(rows, key):
    return np.array([r[key] for r in rows])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv", type=Path)
    parser.add_argument("out", type=Path)
    parser.add_argument("--fine", type=Path)
    parser.add_argument("--equivalence", type=Path)
    parser.add_argument("--circulation", type=Path)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .2})
    groups, phases = load(args.csv, True)
    expected = {connection+"_"+variant for connection in ("wye", "delta") for variant in VARIANTS}
    if set(groups) != expected:
        parser.error("plotter requires the complete 16-case sweep, including both healthy baselines")
    stats = {name: window_stats(rows) for name, rows in groups.items()}
    scored = {}
    for name, row in stats.items():
        baseline = stats[name.split("_")[0]+"_healthy"]
        scored[name] = dict(case=name, **normalize(row, baseline))
    csv_write(args.csv.with_name(args.csv.stem+"_summary.csv"), list(scored.values()))

    fig, axes = plt.subplots(1, 2, figsize=(14, 7), constrained_layout=True)
    for ax, connection in zip(axes, ["wye", "delta"]):
        for y, (variant, label) in enumerate(zip(VARIANTS, LABELS)):
            r = scored[connection+"_"+variant]
            ratio = r["heat_per_work_ratio"]
            if ratio is None:
                ax.text(.1, y, f"NOT SCORED: {r['status']} ({r['heat_j']:.1f} J)", va="center", color="#666666", fontsize=9)
            else:
                color = "#2675bf" if variant == "healthy" else "#d17b28"
                ax.barh(y, ratio, color=color, alpha=.85)
                ax.text(ratio+.05, y, f"{ratio:.2f}×  /  {r['heat_j']:.2f} J", va="center", fontsize=9)
        maximum = max(r["heat_per_work_ratio"] or 0 for n, r in scored.items() if n.startswith(connection+"_"))
        ax.set(xlim=(0, maximum*1.45), title=connection.capitalize(), xlabel="Heat per delivered brake work / healthy baseline")
        ax.axvline(1, color="#444444", ls="--", lw=.8)
        ax.set_yticks(range(len(VARIANTS)), LABELS)
        ax.set_ylim(len(VARIANTS)-.5, -.5)
    fig.suptitle("Correct speed can hide a large electrical-heat regression", fontsize=14)
    fig.supxlabel("Loaded task, 0.5–1.5 s: score requires tracking RMS ≤2 rad/s and brake work within 2% of healthy.\n"
                  "Heat includes copper, FET conduction and diodes; excludes numerical damping, core/switching loss and cooling.", fontsize=9)
    save(fig, args.out, "bldc_heat_regression")

    fig, axes = plt.subplots(2, 3, figsize=(13, 7), sharey="row", constrained_layout=True)
    keys = ["wye_healthy", "wye_high_r", "wye_high_ron"]
    for c, (key, title) in enumerate(zip(keys, ["Healthy wye", "Winding A resistance ×5", "A high FET resistance ↑"])):
        s = stats[key]
        axes[0, c].bar(range(3), [s[f"winding_{p}_heat_j"] for p in "abc"], color=["#d17b28", "#2675bf", "#4b9b69"])
        axes[0, c].set_xticks(range(3), ["A", "B", "C"])
        axes[0, c].set_title(title)
        axes[1, c].bar(range(6), [s[f"fet{j}_heat_j"] for j in range(6)], color=["#777777", "#d17b28"]*3)
        axes[1, c].set_xticks(range(6), ["A low", "A high", "B low", "B high", "C low", "C high"], rotation=30)
        axes[1, c].set_title(f"Diode heat (separate): {s['diode_heat_j']:.3f} J", fontsize=10)
    axes[0, 0].set_ylabel("Winding copper heat (J)")
    axes[1, 0].set_ylabel("FET conduction heat (J)")
    fig.suptitle("Winding and drive faults put heat in different components\nLoaded window 0.5–1.5 s; no temperature prediction", fontsize=13)
    save(fig, args.out, "bldc_heat_sources")

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), sharex=True, constrained_layout=True)
    for c, connection in enumerate(["wye", "delta"]):
        for variant, color, label in zip(["healthy", "advance30", "wrong_sector", "reverse_sector"],
                                         ["#2675bf", "#4b9b69", "#d17b28", "#bb4141"],
                                         ["healthy", "+30° timing", "+60° wrong sector", "+120° wrong sector"]):
            rows = groups[connection+"_"+variant]
            time = values(rows, "time_s")
            axes[0, c].plot(time, values(rows, "speed_rad_s"), color=color, lw=1.1, label=label)
            power = np.gradient(values(rows, "heat_j"), time)
            axes[1, c].plot(time, power, color=color, lw=.8)
        axes[0, c].plot(time, values(rows, "reference_rad_s"), color="black", ls=":", lw=.8)
        axes[0, c].set_title(connection.capitalize())
        axes[0, c].legend(fontsize=8)
        for ax in axes[:, c]:
            ax.axvspan(.5, 1.5, color="gray", alpha=.08)
            ax.set_xlim(0, 2)
        axes[1, c].set_xlabel("Simulated time (s)")
        axes[1, c].set_yscale("symlog", linthresh=1)
    axes[0, 0].set_ylabel("Actual shaft speed (rad/s)")
    axes[1, 0].set_ylabel("Electrical heat rate (W; symlog)")
    fig.suptitle("Wrong commutation can cost heat—or drive the shaft the wrong way\nSame nominal speed PI, bus and task; rotor speed is never assigned by the drive", fontsize=13)
    save(fig, args.out, "bldc_commutation")

    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    phase = np.arange(180)*2+1
    def wave(ax, name, field, color, label, style="-", band=False):
        p = phases[name]
        mean = np.divide(p["total"][field], p["count"], out=np.full(180, np.nan), where=p["count"]>0)
        ax.plot(phase, mean, color=color, ls=style, lw=1.3, label=label)
        if band:
            ax.fill_between(phase, p["low"][field], p["high"][field], color=color, alpha=.12)
    wave(axes[0, 0], "wye_healthy", 0, "#2675bf", "healthy A winding = terminal", band=True)
    wave(axes[0, 0], "wye_high_r", 0, "#d17b28", "high-R A winding = terminal", band=True)
    wave(axes[0, 1], "delta_healthy", 0, "#2675bf", "healthy winding AB")
    wave(axes[0, 1], "delta_healthy", 1, "#2675bf", "healthy terminal A", "--")
    wave(axes[0, 1], "delta_high_r", 0, "#d17b28", "high-R winding AB")
    wave(axes[0, 1], "delta_high_r", 1, "#d17b28", "high-R terminal A", "--")
    wave(axes[1, 0], "wye_healthy", 1, "#2675bf", "healthy terminal A", "--")
    wave(axes[1, 0], "wye_open_high", 1, "#d17b28", "A high FET open", band=True)
    wave(axes[1, 0], "wye_open_winding", 0, "#bb4141", "A winding open")
    wave(axes[1, 1], "delta_healthy", 1, "#2675bf", "healthy terminal A", "--")
    wave(axes[1, 1], "delta_open_high", 1, "#d17b28", "A high FET open: terminal A")
    wave(axes[1, 1], "delta_open_winding", 0, "#bb4141", "AB winding open: winding AB")
    wave(axes[1, 1], "delta_open_winding", 1, "#bb4141", "AB winding open: terminal A", ":")
    for ax, title in zip(axes.flat, ["Wye: high winding resistance", "Delta: branch current ≠ terminal current",
                                    "Open FET ≠ open winding", "Delta: open AB still permits terminal A current"]):
        ax.set(title=title, xlabel="True rotor electrical angle (degrees)", ylabel="Current (A)", xlim=(0, 360))
        ax.legend(fontsize=8)
    fig.suptitle("Fault signatures depend on winding topology and conduction paths", fontsize=14)
    fig.supxlabel("Phase-binned 5 us waveform samples from 1.25–1.29 s; bands are sampled PWM ranges, not uncertainty.\n"
                  "Independent trajectories aligned by rotor angle for diagnosis; no current waveform copied between cases.", fontsize=9)
    save(fig, args.out, "bldc_phase_faults")

    if args.equivalence:
        data = np.genfromtxt(args.equivalence, delimiter=",", names=True)
        fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
        for model, label, color, style in [("simple", "reduced alpha/beta", "#2675bf", "-"),
                                           ("wye", "three-winding wye", "#d17b28", "--"),
                                           ("delta", "equivalent sine delta", "#9049a9", ":")]:
            for ax, quantity in zip(axes, ["speed_rad_s", "torque_nm", "heat_j"]):
                ax.plot(data["time_s"], data[f"{model}_{quantity}"], color=color, ls=style, label=label)
        axes[0].legend(fontsize=9)
        axes[3].semilogy(data["time_s"], np.maximum(1e-16, data["max_current_error_a"]), color="#444444")
        for ax, label in zip(axes, ["Speed (rad/s)", "Torque (Nm)", "Copper heat (J)", "Max current discrepancy (A)"]):
            ax.set_ylabel(label)
        axes[-1].set_xlabel("Simulated time (s)")
        fig.suptitle("Reduced and explicit winding models agree in their shared domain\nSinusoidal PM, symmetric R/L, ideal terminal voltages; independent coupled trajectories", fontsize=13)
        fig.supxlabel("Delta parameters are transformed for terminal equivalence—not identical coils simply rewired.\n"
                      "This does not establish equivalence for PWM/floating-leg simplifications or harmonic delta circulation.", fontsize=9)
        save(fig, args.out, "bldc_model_equivalence")

    if args.circulation:
        data = np.genfromtxt(args.circulation, delimiter=",", names=True)
        fig, axes = plt.subplots(2, 1, figsize=(11, 7), sharex=True, constrained_layout=True)
        axes[0].plot(data["time_s"], data["delta_winding_ab_a"], color="#d17b28", label="delta winding AB")
        axes[0].plot(data["time_s"], data["delta_terminal_a_a"], color="#2675bf", ls="--", label="delta terminal A")
        axes[0].plot(data["time_s"], data["wye_winding_a_a"], color="#555555", ls=":", label="wye winding / terminal A")
        axes[1].plot(data["time_s"], data["delta_heat_j"], color="#d17b28", label="delta winding heat")
        axes[1].plot(data["time_s"], data["wye_heat_j"], color="#555555", ls=":", label="wye winding heat")
        axes[0].set_ylabel("Current (A)"); axes[1].set_ylabel("Copper heat (J)")
        axes[1].set_xlabel("Simulated time (s)")
        for ax in axes: ax.legend()
        fig.suptitle("Zero terminal current does not imply zero winding heat in delta\nTrapezoidal PM, externally driven shaft at 100 rad/s, all FETs OFF; body diodes nonconducting", fontsize=13)
        fig.supxlabel("This is a prescribed-speed electrical bench fixture, not a controller forcing a simulated free rotor.\n"
                      "Internal circulation is outside the two-state balanced reduction; coil inductances omit mutual coupling.", fontsize=9)
        save(fig, args.out, "bldc_delta_circulation")

    if args.fine:
        fine, _ = load(args.fine)
        convergence = []
        for name, rows in groups.items():
            fs = window_stats(fine[name])
            if len(rows) != len(fine[name]) or any(a["time_s"] != b["time_s"] for a, b in zip(rows, fine[name])):
                raise ValueError("refinement CSV overview timestamps differ")
            convergence.append(dict(case=name, coarse_heat_j=stats[name]["heat_j"], fine_heat_j=fs["heat_j"],
                heat_change_pct=100*(fs["heat_j"]/stats[name]["heat_j"]-1),
                max_speed_difference_rad_s=max(abs(a["speed_rad_s"]-b["speed_rad_s"]) for a, b in zip(rows, fine[name])),
                coarse_tracking_rms_rad_s=stats[name]["tracking_rms_rad_s"], fine_tracking_rms_rad_s=fs["tracking_rms_rad_s"],
                numerical_loss_per_heat=stats[name]["numerical_loss_j"]/stats[name]["heat_j"]))
        csv_write(args.csv.with_name(args.csv.stem+"_convergence.csv"), convergence)
    print(f"Wrote BLDC figures to {args.out}; summary and optional convergence CSV beside {args.csv}")


if __name__ == "__main__":
    main()
