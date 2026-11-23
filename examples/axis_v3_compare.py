#!/usr/bin/env python3
"""Compare frozen v3 hypotheses with v2 using recorded effort only; no fitting.

Full captures remain continuous. Windowed increments are diagnostics, not model
resets. All cases are seen development evidence, not fresh physical validation.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from axis_fit_controller_workload import Batch, NAMES, load, select_blocks, sha, save
from axis_model_fit import load as load_effort
from axis_model_v3 import EXTRA, Q, velocity

ROOT = Path(__file__).resolve().parents[1]


def csv_write(path, rows):
    if not rows: return
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0])); writer.writeheader(); writer.writerows(rows)


def native_parameters(fit):
    cfg = json.loads((fit/'candidate-model.json').read_text())
    family = cfg.get('family', 'speed-yield')
    extra = (EXTRA[:3]+['moving_loss_elastic_fraction'] if family == 'slip-yield' else
             EXTRA+(['contact_healing_time_s', 'contact_renewal_distance_rad'] if family == 'age-yield' else []))
    if family not in ('speed-yield', 'slip-yield', 'age-yield') or set(cfg['parameters']) != set(NAMES+tuple(extra)):
        raise ValueError('unknown family/parameter set; never silently drop dynamics')
    source = fit/'source'/dict(zip(['speed-yield','slip-yield','age-yield'],
        ['axis_command_model_v3.c','axis_command_model_v3_slip.c','axis_command_model_v3_age.c']))[family]
    if sha(source) != cfg['kernel_sha256']: raise ValueError('kernel changed after fitting')
    return source, np.array([cfg['parameters'][n] for n in list(NAMES)+extra]), family


def figure(path, t, real, reference, effort, predictions, windows, title, origin='capture'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    colors = {'v2':'tab:orange', 'v3a':'tab:blue', 'v3b':'tab:purple', 'v3c':'tab:green', 'v3d':'tab:red', 'v3e':'tab:cyan', 'v3f':'tab:red'}
    fig, axes = plt.subplots(4, len(windows), figsize=(15, 10), squeeze=False, layout='constrained')
    for col, (name, start, stop) in enumerate(windows):
        mask=(t>=start)&(t<=stop); tx=t[mask]-start
        for label, x in dict(Real=real, **predictions).items():
            color=colors.get(label, 'black')
            axes[0,col].plot(tx,x[mask]/Q,label=label,color=color,lw=1.3)
            axes[1,col].plot(tx,velocity(x)[mask],color=color,lw=1.2)
            if label!='Real':axes[2,col].plot(tx,(x[mask]-real[mask])/Q,color=color,label=label,lw=1.2)
        if np.any(np.isfinite(reference)):
            axes[0,col].plot(tx,reference[mask]/Q,'--',color='0.5',label='Recorded reference')
        axes[3,col].step(tx,effort[mask]*32767,where='post',color='tab:olive')
        axes[0,col].set_title(name)
        for row, ylabel in enumerate([f'Position (counts from {origin} origin)','Velocity (rad/s; same estimator)',
                                      'Prediction minus real (counts)','Recorded effort code']):
            axes[row,col].set_ylabel(ylabel);axes[row,col].grid(alpha=.2)
        axes[0,col].legend(fontsize=8,ncol=2)
        axes[3,col].set_xlabel('Time from displayed window start (s)')
    fig.suptitle(title+'\nSame recorded effort; full continuous replay; no fitted shift or local re-zero',fontsize=11)
    fig.savefig(path,dpi=150);plt.close(fig)
    if np.any(np.isfinite(reference)):
        fig, axes=plt.subplots(2,len(windows),figsize=(15,7),squeeze=False,layout='constrained')
        for col,(name,start,stop) in enumerate(windows):
            mask=(t>=start)&(t<=stop);tx=t[mask]-start
            for label,x in dict(Real=real,**predictions).items():
                axes[0,col].plot(tx,(x[mask]-reference[mask])/Q,color=colors.get(label,'black'),label=label)
                axes[1,col].plot(tx,velocity(x)[mask],color=colors.get(label,'black'))
            axes[0,col].set_yscale('symlog',linthresh=2)
            axes[0,col].set(title=name,ylabel='Position minus common reference (counts; symlog)')
            axes[1,col].set(ylabel='Velocity (rad/s)',xlabel='Time from displayed window start (s)')
            for row in range(2):axes[row,col].grid(alpha=.2)
            axes[0,col].legend(fontsize=8,ncol=2)
        fig.suptitle(title+' — settling detail\nSymlog retains large failures while exposing small tracking errors; no curve is shifted',fontsize=11)
        fig.savefig(path.with_name(path.stem+'-detail.png'),dpi=150);plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fits',type=Path,nargs='+',required=True)
    ap.add_argument('--controller',type=Path,nargs='+',required=True)
    ap.add_argument('--coast',type=Path,nargs='+',required=True,help='Completed effort capture directories')
    ap.add_argument('--allow-conditioned-capture-reset',action='store_true',help='Explicit diagnostic only: override a state-carry candidate initial-state policy')
    ap.add_argument('--include-fitter-training',action='store_true',help='Also check original effort and targeted training cohorts')
    ap.add_argument('--labels',nargs='+',help='Explicit labels for the supplied candidates')
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    if args.labels and (len(args.labels)!=len(args.fits) or len(set(args.labels))!=len(args.labels) or 'v2' in args.labels):
        raise ValueError('candidate labels must be distinct, one per fit, and not v2')
    cfg=json.loads((args.fits[0]/'frozen-v2.json').read_text());p=np.array([cfg['parameters'][n] for n in NAMES])
    models={}; model_inventory={}
    src=args.fits[0]/'source/axis_command_model_v2.c'
    if sha(src)!=cfg['kernel_sha256']:raise ValueError('v2 source mismatch')
    (out/'v2').mkdir();models['v2']=(Batch(src,out/'v2'),p)
    for i,fit in enumerate(args.fits):
        config=json.loads((fit/'candidate-model.json').read_text())
        if config.get('state_carry_fitted',False) and not args.allow_conditioned_capture_reset:
            raise ValueError('candidate requires suite-state carry; use axis_v3_chain.py or explicitly request a conditioned reset diagnostic')
        src,pp,family=native_parameters(fit)
        np.testing.assert_array_equal(pp[2:8],p[2:8])
        refined=json.loads((fit/'candidate-model.json').read_text()).get('moving_gains_refined',False)
        if refined:
            if np.any(pp[:2]<p[:2]*.92) or np.any(pp[:2]>p[:2]*1.08):raise ValueError('gain refinement exceeds registered bounds')
        else:np.testing.assert_array_equal(pp[:2],p[:2])
        name=args.labels[i] if args.labels else 'v3'+chr(ord('a')+i)
        if not name.replace('-','').isalnum():raise ValueError('label must be alphanumeric')
        (out/name).mkdir();models[name]=(Batch(src,out/name),pp)
        model_inventory[name]=dict(fit=str(fit),family=family,config_sha256=sha(fit/'candidate-model.json'),
            dependencies={s.name:sha(s) for s in sorted((fit/'source').glob('*.c'))})
    cases=[]; dev=[]
    for root in args.controller:
        training,development,_=select_blocks(root)
        cases += [(path,'controller-training') for path in training]+[(path,'controller-development') for path in development]
        dev+=development
    promoted={Path(p) for fit in args.fits for p in json.loads((fit/'registered.json').read_text()).get('promoted_seen_coasts',[])}
    cases += [(path,'promoted-mixed-pulse-training' if path in promoted else 'old-mixed-pulse-check') for path in args.coast]
    if args.include_fitter_training:
        old=set();targets=set()
        for fit in args.fits:
            reg=json.loads((fit/'registered.json').read_text());old.update(map(Path,reg['old_sources']))
            for root in map(Path,reg['targeted_sources']):
                registration=json.loads((root/'registered-before-first-start.json').read_text())
                targets.update(root/item['name'] for item in registration['plans'])
        cases += [(path,'original-effort-training') for path in sorted(old-set(args.coast))]
        cases += [(path,'targeted-effort-training') for path in sorted(targets)]
    captures=[];moves=[];inventory={};selected={dev[0],dev[-1],*args.coast}
    for case,family in cases:
        if family.startswith('controller'):
            u,x,inv=load(case);ref=np.repeat(np.fromfile(case/'stream-2.i32le',dtype='<i4').reshape(-1,2)[:,1],2)[:len(u)]*Q
            specs=[]
        else:
            ticks,u,angle,dt,plan,run=load_effort(case)
            if dt!=.0005:raise ValueError('2kHz required')
            x=angle-angle[0];ref=np.full(len(x),np.nan);inv={n:sha(case/n) for n in ['plan.json','get_run.json','report.json','stream-0.i64le']}
            specs=[]
            if family in ('old-mixed-pulse-check','promoted-mixed-pulse-training'):
                registration=json.loads((case.parent/'registered-before-start.json').read_text())
                specs=registration['moves']
            origin=(run['command_start_tick']-ticks[0])/1e6
        inventory[str(case)]=inv;t=np.arange(len(x))*.0005
        uu=np.ascontiguousarray(u[None,:]);ll=np.array([len(u)],dtype=np.int32);predictions={}
        for name,(model,pp) in models.items():
            pred=model.simulate(uu,ll,pp)[0];predictions[name]=pred
            error=pred-x;mask=np.arange(0,len(u),4)
            captures.append(dict(source=str(case),family=family,model=name,
                position_rmse_deg=float(np.sqrt(np.mean(error[mask]**2))*180/np.pi),
                velocity_rmse_rad_s=float(np.sqrt(np.mean((velocity(pred)[mask]-velocity(x)[mask])**2))),
                final_error_counts=float(error[-1]/Q)))
            for j,spec in enumerate(specs):
                start=origin+spec['start']*.02-.025
                stop=origin+(specs[j+1]['start']*.02-.1 if j+1<len(specs) else 12.9)
                a=int(np.searchsorted(t,start));b=min(len(t)-1,int(np.searchsorted(t,stop)))
                moves.append(dict(source=str(case),model=name,move=j+1,label=spec['label'],code=spec['code'],
                    pulse_ms=spec['width']*20,measured_increment_deg=float((x[b]-x[a])*180/np.pi),
                    predicted_increment_deg=float((pred[b]-pred[a])*180/np.pi),
                    error_change_deg=float((error[b]-error[a])*180/np.pi)))
        if case in selected:
            label=(case.parent.parent.name if family.startswith('controller') else case.parent.name)+'--'+case.name
            if family.startswith('controller'):
                changes=np.flatnonzero(np.diff(ref))+1
                windows=[('first recorded reference step',max(0,t[changes[0]]-.03),t[changes[0]]+.4),
                         ('last return and settling',max(0,t[changes[-1]]-.03),t[-1])]
            else:windows=[('first half: launches and coasts',0,6.6),('second half: weak/reversal moves',6.6,t[-1])]
            figure(out/(label+'.png'),t,x,ref,u,predictions,windows,label)
            np.savez_compressed(out/(label+'.npz'),t=t,real=x,reference=ref,effort=u,**predictions)
    aggregate=[]
    for family in sorted({r['family'] for r in captures}):
        for name in models:
            rows=[r for r in captures if r['family']==family and r['model']==name]
            aggregate.append(dict(family=family,model=name,captures=len(rows),
                position_rmse_deg=float(np.sqrt(np.mean([r['position_rmse_deg']**2 for r in rows]))),
                velocity_rmse_rad_s=float(np.sqrt(np.mean([r['velocity_rmse_rad_s']**2 for r in rows])))))
    csv_write(out/'captures.csv',captures);csv_write(out/'maneuvers.csv',moves);csv_write(out/'aggregate.csv',aggregate)
    save(out/'inventory.json',dict(models=model_inventory,inputs=inventory,comparer_sha256=sha(Path(__file__)),
        independent_validation=False,selection='diagnostic comparison only; no model coefficients changed'))
    print(json.dumps(aggregate,indent=2))


if __name__=='__main__':main()
