#!/usr/bin/env python3
"""Plot held-out two-body predictions and training gap profile (Matplotlib)."""
import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('predictions', type=Path)
    parser.add_argument('fit', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with args.predictions.open(newline='') as f:
        rows = list(csv.DictReader(f))
    report = json.loads(args.fit.read_text())
    if not rows:
        parser.error('empty predictions')
    t = [float(r['time']) for r in rows]
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), constrained_layout=True)
    for coordinate, color in [('motor', 'tab:blue'), ('load', 'tab:orange')]:
        measured = [float(r[f'{coordinate}_position']) for r in rows]
        predicted = [float(r[f'{coordinate}_predicted']) for r in rows]
        axes[0].plot(t, measured, color=color, label=f'{coordinate} measured')
        axes[0].plot(t, predicted, '--', color=color, label=f'{coordinate} predicted')
        axes[1].plot(t, [p-m for p, m in zip(predicted, measured)], color=color, label=coordinate)
    axes[0].set(title='Held-out effort replay (initial state from measurements)', ylabel='Position')
    axes[1].set(xlabel='Time [s]', ylabel='Prediction − measurement')
    profile = [p for p in report['gap_profile'] if p['squared_error'] is not None]
    axes[2].plot([p['half_gap'] for p in profile], [p['squared_error'] for p in profile], '.-')
    axes[2].axvline(report['parameters']['backlash_half_gap'], color='tab:red', label='fitted half-gap')
    axes[2].set(xlabel='Backlash half-gap [position units]', ylabel='Training squared error')
    for ax in axes:
        ax.grid(alpha=.3)
        ax.legend()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)
    plt.close(fig)


if __name__ == '__main__':
    main()
