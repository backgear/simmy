#!/usr/bin/env python3
"""Read-only joint coast/controller fits: Stribeck or free-deadband v2.
No hardware/backend imports or motion interface. Uses full continuous captures,
not maneuver resets; actual effort and encoder data only. Gains from the tuner
are not inputs to the objective. All evidence is seen development data.
"""
import argparse
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.optimize import least_squares
from axis_fit_controller_workload import Batch,NAMES,arrays,load,select_blocks,sha,save
from axis_model_fit import load as load_effort


def older(case):
    _,u,x,dt,_,_=load_effort(case)
    if dt!=.0005 or np.ptp(x[:200])>2*2*np.pi/4096:raise ValueError('requires quiet 2kHz episode origin')
    return u,x-x[0],{}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recordings',nargs='+',type=Path)
    ap.add_argument('--coast',action='append',type=Path,required=True)
    ap.add_argument('--family',choices=['stribeck','v2-deadband'],default='stribeck')
    ap.add_argument('--frozen-model',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path);ap.add_argument('--max-nfev',type=int,default=60)
    args=ap.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=False)
    seed=json.loads(args.frozen_model.read_text());p=np.array([seed['parameters'][n] for n in NAMES])
    old_sources=list(map(Path,seed['training_sources']));train=[];dev=[]
    for root in args.recordings:
        a,b,_=select_blocks(root);train+=a;dev+=b
    old_data=[older(c) for c in old_sources];training=[load(c) for c in train]
    development=[load(c) for c in dev];coasts=[older(c) for c in args.coast]
    source=out/'source';source.mkdir()
    for name in ['axis_joint_workload_fit.py','axis_fit_controller_workload.py','axis_model_fit.py',
                 'axis_command_model_v2.c','axis_command_model_stribeck_v1.c']:
        shutil.copy2(Path(__file__).with_name(name),source/name)
    shutil.copy2(args.frozen_model,out/'seed-model.json')
    lower=np.array([5000,5000,50,1,0,0,0,.05]);upper=np.array([25000,25000,600,400,20,20,800,12])
    starts=[np.r_[p[:6],150,1],np.r_[p[:6],250,4],np.r_[p[:6]*[.8,.8,1.2,2,1,1],100,2]]
    if args.family=='v2-deadband':
        lower=np.r_[lower[:6],0.];upper=np.r_[upper[:6],.03]
        starts=[np.r_[p[:6],p[7]],np.r_[p[:6]*[1.05,1.05,1,1,1,1],.012],
                np.r_[p[:6]*[1.1,1.1,1.2,1.2,1,1],.02]]
    kernel='axis_command_model_v2.c' if args.family=='v2-deadband' else 'axis_command_model_stribeck_v1.c'
    save(out/'registered.json',dict(family=args.family,old_training_sources=list(map(str,old_sources)),controller_training_sources=list(map(str,train)),
        controller_development_sources=list(map(str,dev)),old_development_coasts=list(map(str,args.coast)),
        starts=[v.tolist() for v in starts],lower=lower.tolist(),upper=upper.tolist(),
        fixed_filter_tau_s=float(p[6]),fixed_deadband=float(p[7]) if args.family=='stribeck' else None,
        objective='equal family/equal case normalized position + 0.25 normalized 10ms central-difference velocity MSE; linear loss',
        selection='minimum joint training objective; development blocks not used to select starts',
        max_nfev=args.max_nfev,independent_validation=False,source_sha256={c.name:sha(c) for c in source.iterdir()},
        seed_sha256=sha(args.frozen_model)))
    model=Batch(source/kernel,out)
    def prepare(episodes):
        u,x,ll,mask,_=arrays(episodes)
        scale=np.maximum(np.array([np.ptp(e[1]) for e in episodes]),1.)[:,None]
        weights=1/np.sqrt(mask.sum(axis=1)*len(ll))[:,None]/scale
        velocity=(x[:,20:]-x[:,:-20])/.01
        vm=np.zeros_like(velocity,dtype=bool)
        vscale=[]
        for i,length in enumerate(ll):
            vm[i,:length-20:4]=True
            vscale.append(max(float(np.percentile(abs(velocity[i,:length-20]),95)),1.))
        vw=1/np.sqrt(vm.sum(axis=1)*len(ll))[:,None]/np.array(vscale)[:,None]
        return u,x,ll,mask,weights,velocity,vm,vw
    families=[prepare(old_data),prepare(training)]
    def params(free):
        return (np.r_[free[:6],p[6],free[6]] if args.family=='v2-deadband' else np.r_[free[:6],p[6:],free[6:]])
    def residual(free):
        parts=[]
        for u,x,ll,mask,w,velocity,vm,vw in families:
            predicted=model.simulate(u,ll,params(free));parts.append(((predicted-x)*w)[mask])
            pv=(predicted[:,20:]-predicted[:,:-20])/.01
            parts.append(.5*((pv-velocity)*vw)[vm])
        return np.concatenate(parts)
    evaluation={name:arrays(data) for name,data in [('old_training',old_data),('controller_training',training),('controller_development',development),('old_development_coasts',coasts)]}
    def metrics(free):
        result={}
        for name,(u,x,ll,mask,_) in evaluation.items():
            error=model.simulate(u,ll,params(free))-x
            result[name]=float(np.sqrt(np.mean(error[mask]**2))*180/np.pi)
        return result
    results=[]
    for i,start in enumerate(starts):
        fit=least_squares(residual,start,bounds=(lower,upper),x_scale='jac',loss='linear',max_nfev=args.max_nfev,
            ftol=1e-6,xtol=1e-6,gtol=1e-6)
        row=dict(start=i,free_parameters=fit.x.tolist(),cost=float(fit.cost),nfev=fit.nfev,success=bool(fit.success),
            message=str(fit.message),position_rmse_deg=metrics(fit.x))
        results.append(row);save(out/'starts.json',results);print(json.dumps(row),flush=True)
    chosen=min(results,key=lambda r:r['cost']);coeff=params(np.array(chosen['free_parameters']))
    names=list(NAMES)+([] if args.family=='v2-deadband' else ['extra_static_loss_rad_s2','stribeck_velocity_scale_rad_s'])
    result=dict(schema='axis-command-model-v2' if args.family=='v2-deadband' else 'axis-command-model-stribeck-v1',
        status='experimental joint-fit candidate, not promoted or independently validated',parameters=dict(zip(names,map(float,coeff))),
        encoder_counts_per_rev=4096,integration_step_s=.0005,kernel_sha256=sha(source/kernel),
        original_v2_sha256=sha(args.frozen_model),independent_validation=False,
        limitations=['Effective plant coefficients, not calibrated current/torque or intrinsic friction.',
                     'No angle, elastic friction or history state; additional-parameter families need a separate tuner adapter.',
                     'All data seen; source drive-health uncertainty remains. Fresh physical validation required.'])
    save(out/'candidate-model.json',result);save(out/'summary.json',dict(chosen=chosen,old_training_cases=len(old_data),
        controller_training_cases=len(train),controller_development_cases=len(dev),old_development_cases=len(coasts),
        candidate_sha256=sha(out/'candidate-model.json'),independent_validation=False))

if __name__=='__main__':main()
