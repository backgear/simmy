#!/usr/bin/env python3
"""Offline matched-speed coast comparison. No parameter fitting or hardware IO.

Uses centered +/-5ms velocity differences; 20ms coast balances every 10ms,
starting 20ms after pulse ends. Includes negative deceleration windows rather
than discarding cogging/reacceleration. Statistics average within each pulse
first; error bars are SD across pulse means, not confidence intervals.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from axis_coast_speed_trials import extract, write_csv


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bundles',type=Path,nargs='+');ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    windows=[];metrics=[]
    for bundle in args.bundles:
        for case in sorted(bundle.glob('batch-*')):
            m,w=extract(case);metrics.extend(m);windows.extend(w)
    write_csv(args.output/'coast-windows.csv',windows);write_csv(args.output/'pulse-metrics.csv',metrics)
    per_pulse={}
    for w in windows:
        pct=round(w['command_fraction']*100);bin_lo=int(w['speed_rad_s']//5)*5
        key=(pct,w['coast_command_fraction'],w['direction'],bin_lo,w['source'],w['leg'])
        per_pulse.setdefault(key,[]).append(w['deceleration_rad_s2'])
    groups={}
    for (pct,coast,sign,bin_lo,source,leg),values in per_pulse.items():
        groups.setdefault((pct,coast,sign,bin_lo),[]).append(float(np.mean(values)))
    stats=[]
    for (pct,coast,sign,bin_lo),values in sorted(groups.items()):
        stats.append(dict(amplitude_percent=pct,coast_command_fraction=coast,direction=sign,speed_bin_lo_rad_s=bin_lo,
           speed_bin_hi_rad_s=bin_lo+5,pulses=len(values),
           mean_deceleration_rad_s2=float(np.mean(values)),
           std_pulse_mean_deceleration_rad_s2=float(np.std(values,ddof=1)) if len(values)>1 else None))
    write_csv(args.output/'speed-bin-summary.csv',stats)
    seed=json.loads(args.seed.read_text())
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,5),sharey=True)
    for ax,sign,name in zip(axes,(1,-1),('positive','negative')):
        for pct,coast in sorted({(r['amplitude_percent'],r['coast_command_fraction']) for r in stats}):
            color='tab:green' if coast else 'tab:blue' if pct==10 else 'tab:orange'
            marker='^' if coast else 'o' if pct==10 else 's'
            rr=[r for r in stats if r['amplitude_percent']==pct and r['coast_command_fraction']==coast and r['direction']==sign]
            ax.errorbar([r['speed_bin_lo_rad_s']+2.5 for r in rr],
               [r['mean_deceleration_rad_s2'] for r in rr],
               yerr=[r['std_pulse_mean_deceleration_rad_s2'] or 0 for r in rr],
               marker=marker,color=color,label=f'{pct}% launch, {coast*100:+.1f}% coast; SD')
        d=seed['directional'][name];v=np.linspace(5,50,100)
        ax.plot(v,d['coast_coulomb_deceleration_rad_s2']+d['coast_viscous_rate_s_inv']*v,
                '--',color='gray',label='frozen 10% coast seed')
        ax.set_title(f'{name} motion');ax.set_xlabel('Speed magnitude (rad/s)')
        ax.grid(True,alpha=.3);ax.legend(fontsize='small')
    axes[0].set_ylabel('Deceleration magnitude (rad/s²)')
    fig.suptitle('Matched-speed coast-down: larger launches vs earlier baseline')
    fig.tight_layout();fig.savefig(args.output/'coast-speed-comparison.svg');plt.close(fig)
    # Equivalent bias under the hypothesis of symmetric mechanical loss plus
    # constant residual torque; not a measurement or unique identification.
    equivalents=[]
    for lo in sorted({r['speed_bin_lo_rad_s'] for r in stats}):
        rr={r['direction']:r for r in stats if r['amplitude_percent']==20 and r['coast_command_fraction']==0 and r['speed_bin_lo_rad_s']==lo}
        if set(rr)!={1,-1} or min(r['pulses'] for r in rr.values())<4:continue
        ap_,am=rr[1]['mean_deceleration_rad_s2'],rr[-1]['mean_deceleration_rad_s2']
        bias=(am-ap_)/2
        equivalents.append(dict(speed_bin_lo_rad_s=lo,
           symmetric_loss_deceleration_rad_s2=(ap_+am)/2,
           residual_bias_acceleration_rad_s2=bias,
           residual_bias_current_equivalent_A=bias*seed['physical_parameters']['rotor_inertia_kg_m2']/seed['physical_parameters']['torque_constant_Nm_A']))
    summary=dict(pulses=len(metrics),coast_windows=len(windows),
       all_analysis_windows_stationary=all(m['stationary_windows'] for m in metrics),
       speed_bins=stats,conditional_bias_equivalents=equivalents,
       bias_warning='Assumes symmetric mechanical loss and rotor-only inertia. Not measured current; '
                    'directional mechanical friction is also compatible. No coast torque is independently measured.')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(dict(conditional_bias_equivalents=equivalents),indent=2))
    for pct,coast in sorted({(round(abs(m['command_fraction'])*100),m['coast_command_fraction']) for m in metrics}):
        for sign in (1,-1):
            mm=[m for m in metrics if round(abs(m['command_fraction'])*100)==pct and m['coast_command_fraction']==coast and m['direction']==sign]
            print('MOVES',pct,coast,sign,'n',len(mm),'mean_deg',float(np.mean([m['displacement_deg'] for m in mm])),
                  'mean_peak_rad_s',float(np.mean([m['peak_speed_rad_s'] for m in mm])))


if __name__=='__main__':main()
