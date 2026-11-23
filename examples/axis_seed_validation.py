#!/usr/bin/env python3
"""Frozen-seed held-out prediction. No fitting, sample resets, or hardware access.

Ideal rectangular current command plus direction-specific constant/viscous drag,
starting at rest. Ignores RC/current-loop dynamics, cogging, history, elasticity.
Compares nominal physical gain and empirical gain; both use training coast drag.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np


def prediction(t, start, width, fraction, gain, c, b):
    elapsed=np.maximum(0,t-start);on=np.minimum(elapsed,width)
    accel=max(0,gain*abs(fraction)-c)
    if b<1e-8:
        angle=.5*accel*on**2;v0=accel*width
    else:
        angle=accel/b*(on+np.expm1(-b*on)/b)
        v0=accel/b*(-np.expm1(-b*width))
    coast=np.maximum(0,elapsed-width)
    if b<1e-8:
        coast=np.minimum(coast,v0/c if c>0 else np.inf)
        angle+=v0*coast-.5*c*coast**2
    else:
        stop=np.log1p(b*v0/c)/b if c>0 else np.inf
        coast=np.minimum(coast,stop)
        angle+=(v0+c/b)*(-np.expm1(-b*coast))/b-c/b*coast
    return np.sign(fraction)*angle


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--seed',type=Path,required=True);ap.add_argument('--archive',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    seed_bytes=args.seed.read_bytes();seed=json.loads(seed_bytes)
    args.output.mkdir(parents=True,exist_ok=False);(args.output/'seed.json').write_bytes(seed_bytes)
    rows=[];traces=[]
    index=json.loads((args.archive/'dataset-index.json').read_text())
    for bundle in (args.archive/e['bundle'] for e in index if e['role']=='validation'):
        health=json.loads((bundle/'health.json').read_text());hz=health['hello']['tick_hz']
        for case in sorted(bundle.glob('pair-*')):
            report=json.loads((case/'report.json').read_text())
            if not report['passed'] or report['state']!='COMPLETE':raise ValueError('invalid capture')
            plan=json.loads((case/'plan.json').read_text());run=json.loads((case/'get_run.json').read_text())
            desc=run['streams'][0];raw=np.fromfile(case/'stream-0.i64le',dtype='<i8')
            if len(raw)!=desc['sample_count']:raise ValueError('raw count mismatch')
            cp=plan['manifest']['command_period_ticks'];cmd=np.array(plan['commands'])
            active=np.flatnonzero(cmd)
            if len(active)==0 or len(active)!=active[-1]-active[0]+1:raise ValueError('not one pulse')
            fraction=cmd[active[0]]/32767;direction='positive' if fraction>0 else 'negative'
            ticks=desc['first_sample_tick']+np.arange(len(raw),dtype=np.int64)*desc['period_ticks']
            t=(ticks-run['command_start_tick'])/hz
            actual=(raw-raw[0])*2*np.pi/seed['encoder_counts_per_rev']
            start=active[0]*cp/hz;width=len(active)*cp/hz
            d=seed['directional'][direction]
            predictions={}
            for candidate in ('nominal_physical_gain','empirical_gain'):
                gain=(seed['nominal_physical_predictions']['acceleration_gain_rad_s2_per_fullscale']
                      if candidate=='nominal_physical_gain' else
                      d['apparent_command_acceleration_gain_rad_s2_per_full_scale'])
                predicted=prediction(t,start,width,fraction,gain,
                    d['coast_coulomb_deceleration_rad_s2'],d['coast_viscous_rate_s_inv'])
                error=np.rad2deg(predicted-actual);mask=(t>=start)&(t<=start+width+.5)
                rows.append(dict(case=str(case),candidate=candidate,direction=direction,
                    pulse_ms=width*1000,command_fraction=float(fraction),
                    measured_final_deg=float(np.rad2deg(actual[-1])),predicted_final_deg=float(np.rad2deg(predicted[-1])),
                    final_error_deg=float(error[-1]),motion_rmse_deg=float(np.sqrt(np.mean(error[mask]**2)))))
                predictions[candidate]=predicted
            traces.append((direction,width,t,actual,predictions))
    with (args.output/'trial-validation.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    summary=dict(seed_sha256=hashlib.sha256(seed_bytes).hexdigest(),validation_trials=len(traces),
       fit_on_validation=False,initialization='one measured initial position and zero velocity per complete trial',
       limitations=__doc__,candidates={})
    for candidate in ('nominal_physical_gain','empirical_gain'):
        rr=[r for r in rows if r['candidate']==candidate]
        summary['candidates'][candidate]=dict(mean_absolute_final_error_deg=float(np.mean([abs(r['final_error_deg']) for r in rr])),
              mean_motion_rmse_deg=float(np.mean([r['motion_rmse_deg'] for r in rr])))
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,2,figsize=(12,8),sharex=True)
    for direction,width,t,actual,preds in traces:
        ax=axes[0 if direction=='positive' else 1,0 if width<.01 else 1]
        ax.plot(t,np.rad2deg(actual),color='black',alpha=.35)
        for candidate,color in (('nominal_physical_gain','tab:blue'),('empirical_gain','tab:orange')):
            ax.plot(t,np.rad2deg(preds[candidate]),color=color,alpha=.5)
        ax.set_title(f'{direction}, {width*1000:g}ms');ax.grid(True,alpha=.3);ax.set_xlim(-.05,.55)
    axes[0,0].set_ylabel('Relative angle (deg)');axes[1,0].set_ylabel('Relative angle (deg)')
    for ax in axes[1]:ax.set_xlabel('Time from command start (s)')
    fig.suptitle('Reserved trials: measured black; nominal gain blue; empirical gain orange. No fitting.')
    fig.tight_layout();fig.savefig(args.output/'held-out-prediction.svg');plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
