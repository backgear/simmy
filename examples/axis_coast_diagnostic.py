#!/usr/bin/env python3
"""Offline diagnostic, NOT replacement validation: initialize each coast once
from measured velocity to distinguish entry-state error from coast-law error.
No fitting, timing adjustment, hardware access or model changes. Analytic v2
zero-command solution ignores the filter after a 20ms zero-input exclusion.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from axis_model_fit import load


def coast(t,v,c,b):
    speed=abs(v)
    stop=np.log1p(b*speed/c)/b if b>1e-10 else speed/c
    duration=np.minimum(t,stop)
    if b>1e-10:
        angle=(speed+c/b)*(-np.expm1(-b*duration))/b-c/b*duration
    else:angle=speed*duration-.5*c*duration**2
    return np.sign(v)*angle


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('bundles',type=Path,nargs='+');ap.add_argument('--model',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    p=json.loads(args.model.read_text())['parameters'];rows=[];traces=[]
    for bundle in args.bundles:
        case=bundle/'fresh-short-long-braking'
        ticks,u,angle,dt,plan,run=load(case)
        prediction=np.genfromtxt(bundle/'predicted-before-start.csv',delimiter=',',names=True)
        model_x=prediction['v2_position_rad'];model_v=prediction['v2_velocity_rad_s']
        if len(model_x)!=len(angle):raise ValueError('prediction length mismatch')
        idx=np.arange(len(angle));lo=np.maximum(0,idx-10);hi=np.minimum(len(angle)-1,idx+10)
        measured_v=(angle[hi]-angle[lo])/((hi-lo)*dt)
        i=0;number=0
        while i<len(u):
            if u[i]!=0:i+=1;continue
            start=i
            while i<len(u) and u[i]==0:i+=1
            if start==0:continue
            a=start+round(.02/dt);b=min(i,a+round(1.5/dt))
            if b-a<round(.1/dt) or abs(measured_v[a])<3:continue
            number+=1;v0=measured_v[a];direction='positive' if v0>0 else 'negative'
            c=p[direction+'_constant_loss_rad_s2'];visc=p[direction+'_viscous_s_inv']
            tt=np.arange(b-a)*dt;real=angle[a:b]-angle[a]
            analytic=coast(tt,v0,c,visc)
            original=model_x[a:b]-model_x[a]
            e_original=np.rad2deg(original-real);e_law=np.rad2deg(analytic-real)
            rows.append(dict(bundle=bundle.name,coast=number,direction=direction,
              duration_s=float(tt[-1]),measured_entry_velocity_rad_s=float(v0),
              model_entry_velocity_rad_s=float(model_v[a]),
              original_local_position_rmse_deg=float(np.sqrt(np.mean(e_original**2))),
              measured_entry_coast_rmse_deg=float(np.sqrt(np.mean(e_law**2))),
              original_end_error_deg=float(e_original[-1]),measured_entry_end_error_deg=float(e_law[-1])))
            traces.append((real,original,analytic))
    with (args.output/'coast-diagnostic.csv').open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
    error_original=np.concatenate([o-r for r,o,a in traces]);error_law=np.concatenate([a-r for r,o,a in traces])
    summary=dict(status='diagnostic only: not independent whole-trajectory validation',coasts=len(rows),
       minimum_entry_speed_rad_s=3,zero_command_exclusion_ms=20,
       original_coast_displacement_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error_original**2)))),
       measured_entry_coast_displacement_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error_law**2)))),
       interpretation='Measured-entry replay removes incoming velocity error. Remaining drift tests zero-command law, '
                      'including unmeasured residual drive torque, not uniquely mechanical friction.',
       caveats='Measured entry velocity is a centered 10ms estimate. Analytic coast instead of 0.5ms Euler; '
               'filter neglected after 20ms. Local position origin subtracted for diagnostic, no model retuning.')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps(summary,indent=2))
    for r in rows:print(r['bundle'],r['direction'],round(r['measured_entry_velocity_rad_s'],1),
                       round(r['original_end_error_deg'],1),round(r['measured_entry_end_error_deg'],1))


if __name__=='__main__':main()
