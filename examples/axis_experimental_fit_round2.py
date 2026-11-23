#!/usr/bin/env python3
"""Second offline development round; hypotheses informed by seen round-1 errors.
Parameters are still fit only on the exact frozen-v2 training corpus. This is
backtest-driven model development, never independent validation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import time
import numpy as np
from scipy.optimize import least_squares
from axis_experimental_fit import Experimental,training,derivative


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--v2-model',type=Path,required=True);ap.add_argument('--round1',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--stribeck-only',action='store_true')
    ap.add_argument('--joint-compression-only',action='store_true');args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    v2=json.loads(args.v2_model.read_text());episodes,coasts=training(v2['training_sources'])
    if len(episodes)!=v2['training_episodes']:raise ValueError('training corpus changed')
    get=lambda name:json.loads((args.round1/(name+'.json')).read_text())
    first=get('coast_first_linear');spline=get('speed_spline');bristle=get('bristle_memory');phase_model=get('phase_torque')
    common_lo=[5000,5000,.00005,0];common_hi=[23000,23000,.01,.008]
    cvec=first['parameter_vector'];fixed={i:cvec[i] for i in (4,5,6,7)}
    specs={}
    specs['spline_flat_extrapolation']=dict(kind=5,initial=spline['parameter_vector'],
        lower=common_lo+[1]*10,upper=common_hi+[700]*10,labels=spline['parameter_order'],fixed={},objective='balanced')
    specs['coast_velocity_linear']=dict(kind=0,initial=cvec,
        lower=common_lo+[1,1,0,0],upper=common_hi+[500,300,10,10],
        labels=first['parameter_order'],fixed=fixed,objective='driven_velocity_only')
    compressed=list(cvec)+[.6,.6];compressed[0]*=1.1;compressed[1]*=1.1
    specs['coast_velocity_compressed']=dict(kind=6,initial=compressed,
        lower=common_lo+[1,1,0,0,0,0],upper=common_hi+[500,300,10,10,5,5],
        labels=first['parameter_order']+['compression_positive','compression_negative'],fixed=fixed,objective='driven_velocity_only')
    for stiffness in (2000,5000):
        p=list(bristle['parameter_vector']);p[8]=stiffness;p[9]=.025 if stiffness==2000 else .02
        specs[f'soft_bristle_{stiffness}']=dict(kind=4,initial=p,
            lower=common_lo+[30,-.95,0,0,500,.001],upper=common_hi+[350,.95,10,10,30000,.04],
            labels=bristle['parameter_order'],fixed={8:stiffness},objective='balanced')
    p=list(specs['soft_bristle_5000']['initial'])+phase_model['parameter_vector'][8:12]
    specs['phase_bristle_5000']=dict(kind=7,initial=p,
        lower=specs['soft_bristle_5000']['lower']+[-80]*4,
        upper=specs['soft_bristle_5000']['upper']+[80]*4,
        labels=bristle['parameter_order']+['sin1','cos1','sin12','cos12'],fixed={8:5000},objective='balanced')
    if args.stribeck_only:
        linear=json.loads((args.round1/'balanced_linear.json').read_text())
        specs={'classic_stribeck':dict(kind=8,initial=linear['parameter_vector']+[30,30,5],
            lower=common_lo+[1,1,0,0,0,0,.25],upper=common_hi+[500,300,10,10,300,300,15],
            labels=linear['parameter_order']+['Stribeck_extra_positive','Stribeck_extra_negative','Stribeck_speed'],
            fixed={},objective='balanced')}
    if args.joint_compression_only:
        specs={'joint_compressed_linear':dict(kind=6,initial=compressed,
            lower=common_lo+[1,1,0,0,0,0],upper=common_hi+[500,300,10,10,5,5],
            labels=first['parameter_order']+['compression_positive','compression_negative'],
            fixed={},objective='balanced')}
    plan=dict(registered_unix_ns=time.time_ns(),training_sources=v2['training_sources'],training_episodes=len(episodes),
        hypotheses_informed_by_seen_round1_backtests=True,parameters_fit_only_on_v2_training=True,
        independent_validation=False,candidates=specs)
    (args.output/'experiment-plan.json').write_text(json.dumps(plan,indent=2)+'\n')
    n=len(episodes);stride=max(len(e[0]) for e in episodes)
    u=np.zeros((n,stride));y=np.zeros_like(u);mask=np.zeros_like(u,dtype=bool);lengths=[];phases=[]
    for i,(uu,xx,phi,source) in enumerate(episodes):
        u[i,:len(uu)]=uu;y[i,:len(xx)]=xx;mask[i,:len(xx):4]=True;lengths.append(len(xx));phases.append(phi)
    velocity=derivative(y,lengths);norm=np.sqrt(mask.sum(axis=1))[:,None]
    wx=1/np.maximum(np.ptp(y,axis=1),1)[:,None]/norm
    wv=1/np.maximum(np.max(abs(velocity),axis=1),5)[:,None]/norm
    active=mask&(u!=0)&((abs(velocity)>1)|(abs(u)>.03))
    wav=1/np.maximum(np.max(abs(velocity),axis=1),5)[:,None]/np.sqrt(np.maximum(active.sum(axis=1),1))[:,None]
    source=Path(__file__).with_name('axis_experimental_models_stribeck.c' if args.stribeck_only else 'axis_experimental_models_round2.c')
    model=Experimental(args.output,source)
    index=[]
    for name,spec in specs.items():
        initial=np.array(spec['initial']);fixed={int(k):v for k,v in spec['fixed'].items()}
        free=[i for i in range(len(initial)) if i not in fixed]
        def full(q):
            p=initial.copy();p[free]=q
            for k,v in fixed.items():p[k]=v
            return p
        def residual(q):
            x,v=model.simulate(u,lengths,phases,full(q),spec['kind']);vv=derivative(x,lengths)
            if spec['objective']=='driven_velocity_only':return ((vv-velocity)*wav)[active]
            return np.concatenate((((x-y)*wx)[mask],((vv-velocity)*wv)[mask]))
        start=time.monotonic()
        starts=[initial[free]]
        if args.joint_compression_only:
            linear=np.array(get('balanced_linear')['parameter_vector'])
            for k in (.05,2):
                alternate=np.r_[linear,[k,k]];alternate[:2]*=(1+k*.15)
                starts.append(alternate[free])
        fits=[least_squares(residual,q,bounds=(np.array(spec['lower'])[free],np.array(spec['upper'])[free]),
             x_scale='jac',loss='soft_l1',f_scale=.003,max_nfev=120,diff_step=1e-4,
             ftol=1e-7,xtol=1e-7,gtol=1e-7) for q in starts]
        fit=min(fits,key=lambda f:f.cost)
        p=full(fit.x)
        record=dict(schema='axis-experimental-model-v1',name=name,kind=spec['kind'],parameter_order=spec['labels'],
            parameter_vector=list(map(float,p)),parameters=dict(zip(spec['labels'],map(float,p))),
            fixed_training_coast_parameters=fixed,training_sources=v2['training_sources'],training_episodes=n,
            objective=spec['objective'],initial_state='rest; measured initial encoder phase; bristle equilibrium from bias plus phase torque',
            solver=dict(success=bool(fit.success),message=fit.message,nfev=fit.nfev,cost=float(fit.cost),seconds=time.monotonic()-start,
                        multistart_costs=[float(f.cost) for f in fits]),
            kernel_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
            warning='Round-2 hypotheses selected after seen backtests. Training coefficients use only v2 corpus. Not independent validation.')
        path=args.output/(name+'.json');path.write_text(json.dumps(record,indent=2)+'\n')
        index.append(dict(name=name,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        print(name,json.dumps(record['solver']),json.dumps(record['parameters']),flush=True)
    (args.output/'candidate-index.json').write_text(json.dumps(index,indent=2)+'\n')


if __name__=='__main__':main()
