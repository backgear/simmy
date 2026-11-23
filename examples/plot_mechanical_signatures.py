#!/usr/bin/env python3
"""Plot the four mechanical-chain effect signatures under open-loop force.
Usage: python3 examples/plot_mechanical_signatures.py [CSV [OUTPUT_DIR]]
Requires numpy and matplotlib.
"""
import csv
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

source = Path(sys.argv[1] if len(sys.argv) > 1 else "build/mechanical_signatures.csv")
out = Path(sys.argv[2] if len(sys.argv) > 2 else "build")
out.mkdir(parents=True, exist_ok=True)
groups = {}
with source.open(newline="") as f:
    for row in csv.DictReader(f):
        groups.setdefault(row["scenario"], []).append(
            {k: float(v) for k, v in row.items() if k != "scenario"})
groups = {name: {k: np.array([r[k] for r in rows]) for k in rows[0]} for name, rows in groups.items()}

plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22})
fig, axes = plt.subplots(3, 1, figsize=(11, 10), constrained_layout=True)

inertia, friction = groups["inertia_only"], groups["viscous_friction"]
axes[0].plot(inertia["tick"], inertia["motor_velocity"], color="#555555", ls="--", label="inertia only")
axes[0].plot(friction["tick"], friction["motor_velocity"], color="#2675bf", label="+ viscous friction")
axes[0].set(title="Inertia vs. viscous friction, same ping-pong force",
            xlabel="Tick", ylabel="Velocity (counts/tick)")
axes[0].legend(loc="upper right")
axes[0].annotate("sharp triangle wave:\nconstant accel either way", xy=(300, 25),
                 fontsize=8, color="#555555")
axes[0].annotate("rounded, lower peak:\nfriction bleeds off speed", xy=(2700, 5),
                 fontsize=8, color="#2675bf")

stiction = groups["stiction"]
zoom = stiction["tick"] <= 1200
axes[1].plot(stiction["tick"][zoom], stiction["motor_velocity"][zoom], color="#20a080", label="velocity")
twin = axes[1].twinx()
twin.plot(stiction["tick"][zoom], stiction["force"][zoom], color="#d08c13", ls=":", lw=1, label="force")
twin.axhline(1.0, color="#d08c13", ls="--", lw=.8, alpha=.6)
twin.annotate("stictionLimit = 1.0", xy=(850, 1.0), fontsize=8, color="#d08c13")
axes[1].axvspan(0, 251, color="#20a080", alpha=.08)
axes[1].annotate("stuck: velocity exactly 0\nwhile |force| < stictionLimit", xy=(10, 2),
                 fontsize=8, color="#20a080")
axes[1].set(title="Stiction breakaway under a slowly ramped force (first cycle only)",
            xlabel="Tick", ylabel="Velocity (counts/tick)")
twin.set_ylabel("Force")
axes[1].legend(loc="upper left")
twin.legend(loc="upper right")

backlash = groups["backlash"]
axes[2].plot(backlash["tick"], backlash["motor_position"], color="#2675bf", label="motor (body 0)")
axes[2].plot(backlash["tick"], backlash["load_position"], color="#d23d45", label="load (body 1)")
axes[2].set(title="Backlash: load position lags at every reversal",
            xlabel="Tick", ylabel="Position (counts)")
axes[2].legend(loc="upper left")

fig.suptitle("Recognizing mechanical-chain effects under the same open-loop \"ping-pong\" force",
             fontsize=13)
for extension in ("png", "svg"):
    fig.savefig(out / f"mechanical_signatures.{extension}", dpi=160)
plt.close(fig)
print(f"Wrote {out}/mechanical_signatures.{{png,svg}}")
