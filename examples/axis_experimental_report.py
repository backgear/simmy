#!/usr/bin/env python3
"""Combine immutable offline backtests, retaining failures and held-out tradeoffs."""
import argparse
import csv
import gzip
import io
import json
from pathlib import Path
import numpy as np
from axis_model_fit import load
from axis_experimental_fit import Experimental,derivative


def savecsv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--root',type=Path,required=True);ap.add_argument('--bundles-root',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    captures={};maneuvers={}
    directories=['backtest','round2-backtest','stribeck-backtest']
    if (args.root/'joint-backtest').exists():directories.append('joint-backtest')
    for d in directories:
        for row in csv.DictReader((args.root/d/'capture-metrics.csv').open()):
            key=(row['bundle'],row['model'])
            if key in captures and captures[key]!=row:raise ValueError('baseline results changed between rounds')
            captures[key]=row
        for row in csv.DictReader((args.root/d/'maneuver-metrics.csv').open()):
            key=(row['bundle'],row['model'],row['maneuver'])
            if key in maneuvers and maneuvers[key]!=row:raise ValueError('baseline maneuver changed')
            maneuvers[key]=row
    models=sorted(set(r['model'] for r in captures.values()));summary=[]
    for model in models:
        row=dict(model=model)
        for split in ('random','designed','all'):
            c=[r for r in captures.values() if r['model']==model and (split=='all' or r['split']==split)]
            m=[r for r in maneuvers.values() if r['model']==model and (split=='all' or r['split']==split)]
            row[split+'_position_rmse_deg']=float(np.sqrt(np.mean([float(r['position_rmse_deg'])**2 for r in c])))
            row[split+'_maneuver_mae_deg']=float(np.mean([abs(float(r['error_deg'])) for r in m]))
            row[split+'_displacement_passes']=sum(r['displacement_pass']=='True' for r in m)
            row[split+'_peak_passes']=sum(r['peak_speed_pass']=='True' for r in m)
        summary.append(row)
    summary.sort(key=lambda r:r['all_position_rmse_deg'])
    savecsv(args.output/'comparison.csv',summary);savecsv(args.output/'capture-metrics.csv',list(captures.values()))
    savecsv(args.output/'maneuver-metrics.csv',list(maneuvers.values()))
    results=dict(status='offline seen-data development backtests; not new independent validation',
        training_episodes=100,training_captures=21,experimental_candidates=len(models)-2,
        evaluation_captures=4,evaluation_maneuvers=24,random_maneuvers=18,
        parameter_fits_only_v2_training_sources=True,v2_unchanged=True,
        all_capture_rmse_winner=summary[0]['model'],
        random_rmse_winner=min(summary,key=lambda r:r['random_position_rmse_deg'])['model'],
        recommendation='Keep v2 frozen. No candidate beats its full-capture position RMSE across all four runs. '
                       'Soft bristle and phase+bristle are hypotheses, not validated replacements.',
        models=summary)
    (args.output/'comparison.json').write_text(json.dumps(results,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    selected=['v2','balanced_linear','bristle_memory','soft_bristle_5000','phase_bristle_5000']
    fig,ax=plt.subplots(figsize=(12,5));xx=np.arange(len(selected))
    for shift,split,color in ((-.25,'random','tab:blue'),(0,'designed','tab:orange'),(.25,'all','tab:green')):
        ax.bar(xx+shift,[next(r for r in summary if r['model']==m)[split+'_position_rmse_deg'] for m in selected],
               width=.25,label=split,color=color)
    ax.set_xticks(xx,selected,rotation=12);ax.set_ylabel('Continuous position RMSE (deg)');ax.legend()
    ax.set_title('Experimental fits: random-only improvement does not carry across every capture')
    ax.grid(True,axis='y',alpha=.3);fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(args.output/('selected-comparison.'+ext),dpi=150)
    plt.close(fig)
    source=Path(__file__).with_name('axis_experimental_models_round2.c');kernel=Experimental(args.output/'kernel',source)
    configs={name:json.loads((args.root/'round2-fit'/(name+'.json')).read_text())
             for name in ('soft_bristle_5000','phase_bristle_5000')}
    for bundle_name in sorted(set(r['bundle'] for r in captures.values())):
        bundle=args.bundles_root/bundle_name;ticks,u,angle,dt,plan,run=load(bundle/'fresh-short-long-braking')
        actual=angle-angle[0];tt=(ticks-run['command_start_tick'])/1e6
        registration=json.loads((bundle/'registered-before-start.json').read_text())
        ref=np.genfromtxt(bundle/'predicted-before-start.csv',delimiter=',',names=True)
        predictions={'v2':ref['v2_position_rad']}
        for name,config in configs.items():
            x,v=kernel.simulate(u[None,:],[len(u)],[angle[0]%(2*np.pi)],config['parameter_vector'],config['kind'],dt)
            predictions[name]=x[0]
        vv={name:derivative(x[None,:],[len(x)])[0] for name,x in predictions.items()}
        mv=derivative(actual[None,:],[len(actual)])[0]
        with (args.output/(bundle_name+'-leading-traces.csv.gz')).open('wb') as raw:
            with gzip.GzipFile(filename='',fileobj=raw,mode='wb',mtime=0) as gz:
                with io.TextIOWrapper(gz,newline='') as f:
                    writer=csv.writer(f);writer.writerow(['time_s','command_fraction','measured_position_rad','measured_velocity_10ms_rad_s']+
                        [s for name in predictions for s in (name+'_position_rad',name+'_velocity_10ms_rad_s')])
                    for i in range(len(u)):
                        writer.writerow([tt[i],u[i],actual[i],mv[i]]+[z for name in predictions for z in (predictions[name][i],vv[name][i])])
        fig,axes=plt.subplots(2,3,figsize=(15,8),squeeze=False)
        for j,m in enumerate(registration['moves']):
            start=m['start']*.02;stop=registration['moves'][j+1]['start']*.02 if j<5 else 12
            a=int(np.searchsorted(tt,start-.025));b=int(np.searchsorted(tt,stop-.1));ax=axes.flat[j]
            ax.plot(tt[a:b]-start,np.rad2deg(actual[a:b]-actual[a]),color='black',label='Measured')
            for name,color in (('v2','tab:orange'),('soft_bristle_5000','tab:blue'),('phase_bristle_5000','tab:green')):
                x=predictions[name];ax.plot(tt[a:b]-start,np.rad2deg(x[a:b]-x[a]),'--',color=color,label=name)
            ax.set_title(m['label']);ax.set_ylabel('Displacement (deg)');ax.set_xlabel('Time from launch (s)')
            ax.grid(True,alpha=.3);ax.legend(fontsize=7)
        fig.suptitle(bundle_name+' — local display origins only; all model states continuous')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/(bundle_name+'-maneuvers.'+ext),dpi=150)
        plt.close(fig)
    for r in summary:
        print(r['model'],*(round(r[s+'_position_rmse_deg'],2) for s in ('random','designed','all')),
              str(r['random_displacement_passes'])+'/18',str(r['random_peak_passes'])+'/18')


if __name__=='__main__':main()
