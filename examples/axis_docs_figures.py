#!/usr/bin/env python3
"""Render compact controller and optional recorded-effort replay figures."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from axis_pc_compare import load, step_metrics


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sim',type=Path,required=True)
    ap.add_argument('--hardware',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--replay',type=Path,help='Optional axis_workload_replay_check case NPZ')
    args=ap.parse_args()
    sim=load(args.sim);real=load(args.hardware)
    if sim[0]['backend']!='sim' or real[0]['backend']!='hardware':raise ValueError('backend labels reversed')
    for key in ('controller_identity','frozen_model_sha256','validation_references','pre_roll_ms'):
        if sim[0][key]!=real[0][key]:raise ValueError('incompatible runs: '+key)
    np.testing.assert_array_equal(sim[5],real[5])
    args.output.mkdir(parents=True,exist_ok=True)
    end=real[0]['pre_roll_ms']+20*len(real[0]['validation_references'])
    t=np.arange(end+1)/1000
    fig,(position,settle)=plt.subplots(2,1,figsize=(11,6),gridspec_kw=dict(height_ratios=[2,1]),layout='constrained')
    position.step(t,real[5][:end+1]*360/4096,where='post',color='0.5',lw=1,label='Requested position')
    position.plot(t,real[3][:end+1]*360/4096,label='Real motor',color='tab:blue')
    position.plot(t,sim[3][:end+1]*360/4096,'--',label='Frozen model',color='tab:orange')
    position.set(xlabel='Capture time (s)',ylabel='Position (degrees)',title='Same controller and PC tuner; independent feedback')
    position.legend();position.grid(alpha=.2)
    width=.35
    for run,shift,label,color in [(sim,-width/2,'Frozen model','tab:orange'),(real,width/2,'Real motor','tab:blue')]:
        metrics=step_metrics(run[3],run[5],end);steps=np.arange(len(metrics))
        settle.bar(steps+shift,[s['settling_2_percent_ms'] for s in metrics],width,label=label,color=color)
    changes=np.flatnonzero(np.diff(real[5]))+1
    labels=[f'{real[5][i-1]:g} → {real[5][i]:g}' for i in changes]
    settle.set(xticks=steps,xticklabels=labels,ylabel='2% settling (ms)',xlabel='Validation step (encoder counts)')
    settle.grid(axis='y',alpha=.2);settle.legend()
    fig.savefig(args.output/'controller-development.png',dpi=140);plt.close(fig)
    if args.replay:
        with np.load(args.replay) as case:
            t=np.arange(len(case['measured_rad']))*float(case['dt']);degrees=180/np.pi
            fig,axes=plt.subplots(3,1,figsize=(11,6),sharex=True,gridspec_kw=dict(height_ratios=[1,2,1]),layout='constrained')
            axes[0].step(t,case['effort']*100,where='post',color='0.4')
            axes[0].set(ylabel='Effort code (% FS)',title='Recorded input replay: predict motion, without position feedback')
            axes[1].plot(t,case['measured_rad']*degrees,label='Measured shaft')
            axes[1].plot(t,case['predicted_rad'][0]*degrees,'--',label='First model prediction')
            axes[1].set_ylabel('Position (degrees)');axes[1].legend()
            axes[2].plot(t,(case['predicted_rad'][0]-case['measured_rad'])*degrees,color='tab:red')
            axes[2].axhline(0,color='0.5',lw=.7)
            axes[2].set(xlabel='Capture time (s)',ylabel='Prediction error (°)')
            for ax in axes:ax.grid(alpha=.2)
            fig.savefig(args.output/'recorded-motion.png',dpi=140);plt.close(fig)


if __name__=='__main__':main()
