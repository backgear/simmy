#!/usr/bin/env python3
"""Continuous offline backtests of frozen experimental fits on seen recordings.

No fitting and no state resets. Randomized records are already seen evaluation
material, so ranking here is model-development backtesting, not fresh validation.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import numpy as np
from axis_model_fit import load
from axis_experimental_fit import Experimental,derivative


def write_csv(path,rows):
    with Path(path).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fits',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--kernel-source',type=Path)
    ap.add_argument('bundles',nargs='+',type=Path);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);kernel=Experimental(args.output/'kernel',args.kernel_source)
    source=args.kernel_source or Path(__file__).with_name('axis_experimental_models.c')
    kernel_sha256=hashlib.sha256(source.read_bytes()).hexdigest()
    index=json.loads((args.fits/'candidate-index.json').read_text());candidates={}
    for row in index:
        path=args.fits/row['file']
        if hashlib.sha256(path.read_bytes()).hexdigest()!=row['sha256']:raise ValueError('candidate changed after fit')
        config=json.loads(path.read_text())
        if config['kernel_sha256']!=kernel_sha256:raise ValueError('candidate/kernel version mismatch')
        candidates[row['name']]=config
    metrics=[];legs=[];traces={}
    for bundle in args.bundles:
        ticks,u,angle,dt,plan,run=load(bundle/'fresh-short-long-braking')
        if dt!=.0005:raise ValueError('backtest currently requires 2kHz')
        registration=json.loads((bundle/'registered-before-start.json').read_text())
        reference=np.genfromtxt(bundle/'predicted-before-start.csv',delimiter=',',names=True)
        actual=angle-angle[0];mv=derivative(actual[None,:],[len(actual)])[0]
        predictions={name:reference[name+'_position_rad'] for name in ('v1','v2')}
        for name,config in candidates.items():
            x,v=kernel.simulate(u[None,:],[len(u)],[angle[0]%(2*np.pi)],
                config['parameter_vector'],config['kind'],dt)
            predictions[name]=x[0]
        tt=(ticks-run['command_start_tick'])/1e6;moving=abs(mv)>1
        coast_mask=moving&(u==0);drive_mask=moving&(u!=0)
        split='random' if registration['random_seed'] is not None else 'designed'
        for name,x in predictions.items():
            pv=derivative(x[None,:],[len(x)])[0];err=np.rad2deg(x-actual)
            metrics.append(dict(bundle=bundle.name,split=split,model=name,
                position_rmse_deg=float(np.sqrt(np.mean(err**2))),
                position_max_error_deg=float(np.max(abs(err))),final_error_deg=float(err[-1]),
                velocity_rmse_rad_s=float(np.sqrt(np.mean((pv-mv)**2))),
                moving_velocity_rmse_rad_s=float(np.sqrt(np.mean((pv[moving]-mv[moving])**2))),
                coast_velocity_rmse_rad_s=float(np.sqrt(np.mean((pv[coast_mask]-mv[coast_mask])**2))),
                driven_velocity_rmse_rad_s=float(np.sqrt(np.mean((pv[drive_mask]-mv[drive_mask])**2)))))
            for j,m in enumerate(registration['moves']):
                start=m['start']*.02;stop=registration['moves'][j+1]['start']*.02 if j+1<len(registration['moves']) else 12
                a=int(np.searchsorted(tt,start-.025));b=int(np.searchsorted(tt,stop-.1))
                real=float(np.rad2deg(actual[b]-actual[a]));sim=float(np.rad2deg(x[b]-x[a]))
                peak=float(np.max(abs(mv[a:b])));pp=float(np.max(abs(pv[a:b])))
                legs.append(dict(bundle=bundle.name,split=split,model=name,maneuver=j+1,
                    label=m['label'],command_fraction=m['code']/32767,pulse_ms=m['width']*20,
                    measured_deg=real,predicted_deg=sim,error_deg=sim-real,
                    displacement_pass=bool(abs(sim-real)<=max(5,.1*abs(real))),
                    peak_speed_pass=bool(abs(pp-peak)<=.15*peak)))
        traces[bundle.name]=(tt,u,actual,mv,predictions,registration)
    write_csv(args.output/'capture-metrics.csv',metrics);write_csv(args.output/'maneuver-metrics.csv',legs)
    aggregate=[]
    for split in ('random','designed'):
        for name in ('v1','v2',*candidates):
            mm=[r for r in metrics if r['model']==name and r['split']==split]
            ll=[r for r in legs if r['model']==name and r['split']==split]
            aggregate.append(dict(split=split,model=name,captures=len(mm),maneuvers=len(ll),
                position_rmse_deg=float(np.sqrt(np.mean([m['position_rmse_deg']**2 for m in mm]))),
                mean_absolute_maneuver_error_deg=float(np.mean([abs(l['error_deg']) for l in ll])),
                displacement_passes=sum(l['displacement_pass'] for l in ll),peak_speed_passes=sum(l['peak_speed_pass'] for l in ll),
                mean_coast_velocity_rmse_rad_s=float(np.mean([m['coast_velocity_rmse_rad_s'] for m in mm])),
                mean_driven_velocity_rmse_rad_s=float(np.mean([m['driven_velocity_rmse_rad_s'] for m in mm]))))
    write_csv(args.output/'aggregate.csv',aggregate)
    ranking=sorted([r for r in aggregate if r['split']=='random'],key=lambda r:r['position_rmse_deg'])
    best=ranking[0]['model']
    summary=dict(status='seen-data backtest/model-development comparison, NOT new independent validation',
       no_fit_on_evaluation_samples=True,training_exactly_v2_sources=True,continuous_state=True,
       candidate_selection_used_seen_backtests=True,best_by_random_position_rmse=best,random_ranking=ranking,
       warning='Selecting a candidate or hyperparameters using these results requires a new capture for independent validation.')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,ax=plt.subplots(figsize=(12,5));ax.bar([r['model'] for r in ranking],[r['position_rmse_deg'] for r in ranking])
    ax.tick_params(axis='x',rotation=25);ax.set_ylabel('Pooled random position RMSE (deg)')
    ax.set_title('Seen randomized backtests; all fits use only v2 training observations')
    ax.grid(True,axis='y',alpha=.3);fig.tight_layout();fig.savefig(args.output/'candidate-comparison.png',dpi=150);plt.close(fig)
    for bundle,(tt,u,actual,mv,predictions,registration) in traces.items():
        fig,axes=plt.subplots(4,1,figsize=(13,10),sharex=True)
        for ax,key,label in ((axes[0],0,'Position (deg)'),(axes[1],1,'Velocity (rad/s)')):
            ax.plot(tt,np.rad2deg(actual) if key==0 else mv,color='black',label='Measured')
            for name,color in (('v2','tab:orange'),(best,'tab:green')):
                x=predictions[name];y=np.rad2deg(x) if key==0 else derivative(x[None,:],[len(x)])[0]
                ax.plot(tt,y,'--',color=color,label=name)
            ax.set_ylabel(label);ax.legend()
        axes[2].step(tt,u*100,where='post');axes[2].set_ylabel('Command (% FS)')
        for name,color in (('v2','tab:orange'),(best,'tab:green')):
            axes[3].plot(tt,np.rad2deg(predictions[name]-actual),color=color,label=name)
        axes[3].set_ylabel('Sim - real (deg)');axes[3].legend();axes[3].set_xlabel('Time (s)')
        for ax in axes:ax.grid(True,alpha=.3)
        fig.suptitle(f'{bundle}: seen backtest, continuous states, no fitted time shifts')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/(bundle+'.'+ext),dpi=150)
        plt.close(fig)
    print(json.dumps(summary,indent=2))


if __name__=='__main__':main()
