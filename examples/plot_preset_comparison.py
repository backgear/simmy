#!/usr/bin/env python3
"""Compare bench and simulated readme sequences (JSON frame lists).

Optional tooling: needs matplotlib and numpy. It reads recordings only;
there is no serial interface and it does not run or retune a controller.
Windows are tick indices at 1 kHz, not host timestamps.
"""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--bench', type=Path, required=True)
    ap.add_argument('--sim', type=Path, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--title', required=True)
    ap.add_argument('--step', nargs=2, type=int, required=True, metavar=('START','END'))
    ap.add_argument('--ramp', nargs=2, type=int, required=True, metavar=('START','END'))
    ap.add_argument('--clip-step-error', type=float, default=60)
    args = ap.parse_args()
    bench = json.loads(args.bench.read_text())
    sim = json.loads(args.sim.read_text())
    if len(bench) != len(sim):
        ap.error('frame counts differ; align the replays before plotting')
    if not all(0 <= lo < hi <= len(bench) for lo, hi in (args.step,args.ramp)):
        ap.error('window outside frame range')
    plt.rcParams.update({'figure.facecolor':'#fcfcfb','axes.facecolor':'#fcfcfb',
                         'axes.spines.top':False,'axes.spines.right':False,
                         'axes.labelcolor':'#52514e','grid.color':'#e1e0d9','font.size':10})
    rows = [('Following error (motor counts)',lambda f:f['cmd_position']-f['position']),
            ('Velocity (counts / tick)',lambda f:f['velocity']),
            ('Torque command (firmware units)',lambda f:f['torque'])]
    fig,axes = plt.subplots(3,2,figsize=(13,8.5),sharex='col',constrained_layout=True)
    for col,(lo,hi) in enumerate((args.step,args.ramp)):
        t = np.arange(hi-lo)/1000
        for row,(label,value) in enumerate(rows):
            ax = axes[row,col]
            ax.axhline(0,color='#bbbbbb',linewidth=.8)
            for name,frames,color in [('Bench',bench,'#2a78d6'),('SIMULATED canonical preset',sim,'#eb6834')]:
                ax.plot(t,[value(f) for f in frames[lo:hi]],color=color,linewidth=1.1,alpha=.85,label=name)
            ax.set_ylabel(label);ax.grid(axis='y');ax.legend(frameon=False,fontsize=8)
        axes[-1,col].set_xlabel('Time from window start (s)')
    axes[0,0].set_title('Unramped position step out and back')
    axes[0,1].set_title('Velocity triangle')
    if args.clip_step_error > 0:
        v = args.clip_step_error
        axes[0,0].set_ylim(-v,v)
        axes[0,0].text(.01,.03,'Error clipped to ±%g counts; command step is ±1024'%v,
                       transform=axes[0,0].transAxes,fontsize=8)
    for ax in axes[2]:ax.set_ylim(-33000,33000)
    fig.suptitle(args.title,fontsize=12,fontweight='bold')
    args.out.parent.mkdir(parents=True,exist_ok=True)
    fig.savefig(args.out,dpi=150)
    plt.close(fig)

if __name__ == '__main__':
    main()
