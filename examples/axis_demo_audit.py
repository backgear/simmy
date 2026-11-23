#!/usr/bin/env python3
"""Compare diagnostic demo variants without fitting parameters or time offsets."""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('directory',type=Path)
    ap.add_argument('--figure',type=Path,required=True);args=ap.parse_args()
    data={name:json.loads((args.directory/(name+'.json')).read_text()) for name in ['legacy','no-reference','corrected']}
    schedule=lambda d:[(f['seq'],f['cmd_position'],f['cmd_velocity']) for f in d]
    assert all(schedule(d)==schedule(data['legacy']) for d in data.values())
    metrics={}
    for name,d in data.items():
        ramp=[f for f in d if f['phase'].startswith('ramp-')]
        hold=[f for f in d if f['phase']=='step' and f['seq']>=620]
        crossings={}
        for count in [512,1024,2048]:
            command=next(f['seq'] for f in ramp if f['cmd_position']>=1024+count)
            actual=next(f['seq'] for f in ramp if f['position']>=1024+count)
            crossings[str(count)]=actual-command
        moving=[f for f in ramp if f['cmd_velocity']>=14]
        metrics[name]=dict(crossing_lag_ms=crossings,ramp_position_mae_counts=float(np.mean([abs(f['cmd_position']-f['position']) for f in moving])),
            inconsistent_velocity_samples=sum(f['velocity']!=f['encoder_delta'] for f in d),
            late_step_position_span_counts=int(np.ptp([f['position'] for f in hold])),
            late_step_integral_span_codes=float(np.ptp([f['integral'] for f in hold])))
    fig,axes=plt.subplots(2,2,figsize=(12,7),layout='constrained')
    for name,label in [('legacy','Old demo'),('corrected','Encoder delta + velocity reference')]:
        d=data[name];ramp=[f for f in d if f['phase'].startswith('ramp-')];origin=ramp[0]['t'];base=ramp[0]['cmd_position']
        axes[0,0].plot([f['t']-origin for f in ramp],[f['position']-base for f in ramp],label=label)
        hold=[f for f in d if f['phase']=='step' and f['seq']>=220]
        axes[0,1].plot([f['t']-.02 for f in hold],[f['cmd_position']-f['position'] for f in hold],label=label)
    axes[0,0].plot([f['t']-origin for f in ramp],[f['cmd_position']-base for f in ramp],'--',color='0.4',label='Same requested ramp')
    axes[0,0].set(xlim=(0,.3),xlabel='Time from common ramp schedule start (s)',ylabel='Position increment (counts)',title='No clock shift: restore the missing velocity reference')
    axes[0,1].set(xlabel='Time from step request (s)',ylabel='Position error (counts)',title='Feedback must agree with encoder position changes')
    corrected=data['corrected'];hold=[f for f in corrected if f['phase']=='step' and f['seq']>=220]
    for key,label in [('torque','Applied effort code'),('integral','Stored integral contribution'),('proportional','P contribution')]:
        axes[1,0].plot([f['t']-.02 for f in hold],[f[key] for f in hold],label=label)
    axes[1,0].set(xlabel='Time from step request (s)',ylabel='Effort code',title='Remaining integral drift: one count of residual error')
    ramp=[f for f in corrected if f['phase'].startswith('ramp-')];origin=ramp[0]['t']
    unsaturated=[f for f in ramp if abs(f['unclamped_candidate_torque'])<3276]
    axes[1,1].plot([f['t']-origin for f in unsaturated],[f['torque_minus_stored_integral'] for f in unsaturated],label='Applied output − stored I')
    axes[1,1].plot([f['t']-origin for f in unsaturated],[f['proportional'] for f in unsaturated],'--',label='Computed P contribution')
    axes[1,1].set(xlabel='Time from common ramp schedule start (s)',ylabel='Effort code',title='Integral-adjusted output is informative away from clipping')
    for ax in axes.flat:ax.grid(alpha=.2);ax.legend(fontsize=8)
    fig.suptitle('Simulation harness audit · identical frozen plant, gains and command schedule')
    args.figure.parent.mkdir(parents=True,exist_ok=True);fig.savefig(args.figure,dpi=140);plt.close(fig)
    report=dict(metrics=metrics,time_shift_fitted=False,plant_refitted=False,hardware_run=False,
        raw_sha256={name:hashlib.sha256((args.directory/(name+'.json')).read_bytes()).hexdigest() for name in data})
    (args.directory/'audit.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report,indent=2))

if __name__=='__main__':main()
