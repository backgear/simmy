#!/usr/bin/env python3
"""Offline v2 fit: separate +/- command gains and constant/viscous drag.

Seen validation may be explicitly promoted to training; never claim it remains
independent validation. The next novel capture must be frozen/predicted first.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.optimize import least_squares
from axis_model_fit import Model as ModelV1,load

NAMES=['positive_gain_rad_s2_per_command','negative_gain_rad_s2_per_command',
       'positive_constant_loss_rad_s2','negative_constant_loss_rad_s2',
       'positive_viscous_s_inv','negative_viscous_s_inv',
       'command_filter_tau_s','command_deadband_fraction']


class Model(ModelV1):
    def __init__(self,build):
        build=Path(build);build.mkdir(parents=True,exist_ok=True)
        source=Path(__file__).with_name('axis_command_model_v2.c')
        library=(build/'axis_command_model_v2.so').resolve()
        subprocess.run(['cc','-O3','-shared','-fPIC',str(source),'-lm','-o',str(library)],check=True)
        self.lib=ctypes.CDLL(str(library))
        self.lib.axis_model.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,
                                     ctypes.c_int,ctypes.c_double,ctypes.c_void_p,
                                     ctypes.c_void_p,ctypes.c_void_p]
        self.lib.axis_model.restype=None


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('training_bundles',nargs='+',type=Path)
    ap.add_argument('--promote-seen-validation',action='store_true')
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    episodes=[];sources=[];promoted=[]
    for bundle in args.training_bundles:
        if 'validation' in bundle.name:
            if not args.promote_seen_validation:raise ValueError('explicit seen-validation promotion required')
            promoted.append(str(bundle))
        for case in sorted(p for p in bundle.iterdir() if p.is_dir() and (p/'report.json').exists()):
            ticks,u,angle,dt,plan,run=load(case)
            if dt!=.0005:raise ValueError('requires 2kHz training')
            cmd=plan['commands'];cp=plan['manifest']['command_period_ticks'];origin=run['command_start_tick']
            active=[];i=0
            while i<len(cmd):
                if abs(cmd[i])<3276:i+=1;continue
                a=i;code=cmd[i]
                while i<len(cmd) and cmd[i]==code:i+=1
                active.append((origin+a*cp,origin+i*cp))
            for j,(a,b) in enumerate(active):
                end=min(b+1100000,active[j+1][0]-50000 if j+1<len(active) else ticks[-1])
                mask=(ticks>=a-50000)&(ticks<=end)
                xx=angle[mask];uu=u[mask]
                if np.ptp(xx[:80])>2*2*np.pi/4096:raise ValueError('nonquiet start')
                episodes.append((uu,xx-xx[0]));sources.append(str(case))
    stride=max(len(u) for u,x in episodes);n=len(episodes)
    uu=np.zeros((n,stride));yy=np.zeros_like(uu);mask=np.zeros_like(uu,dtype=bool)
    lengths=[];weights=[]
    for i,(u,x) in enumerate(episodes):
        uu[i,:len(u)]=u;yy[i,:len(x)]=x;mask[i,:len(x):4]=True
        lengths.append(len(u));weights.append(1/max(np.ptp(x),1.)/np.sqrt(np.sum(mask[i])))
    weights=np.array(weights)[:,None];model=Model(args.output)
    def residual(p):
        pred,_=model.simulate(uu,lengths,.0005,p)
        return ((pred-yy)*weights)[mask]
    initial=[12500,12500,250,70,.2,.7,.001,.002]
    lower=[5000,5000,50,1,0,0,.00005,0]
    upper=[20000,20000,450,250,8,8,.01,.008]
    fit=least_squares(residual,initial,bounds=(lower,upper),x_scale='jac',loss='soft_l1',
                      f_scale=.003,max_nfev=150,ftol=1e-7,xtol=1e-7,gtol=1e-7)
    source=Path(__file__).with_name('axis_command_model_v2.c')
    result=dict(schema='axis-command-model-v2',status='frozen before next fresh validation',
       model='filtered deadbanded command with separate positive/negative effective constant and viscous drag; stiction',
       units='mechanical radians/seconds; command normalized to code/32767, not measured current',
       encoder_counts_per_rev=4096,integration_step_s=.0005,parameters=dict(zip(NAMES,map(float,fit.x))),
       training_episodes=n,training_sources=sorted(set(sources)),seen_validation_promoted_to_training=promoted,
       kernel_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
       solver=dict(success=bool(fit.success),message=fit.message,nfev=fit.nfev,cost=float(fit.cost)),
       limitations=['constant directional losses absorb any residual torque bias; no unique electrical offset',
                    'no rotor-angle/cogging/hysteresis state','effective filter/gains are not independently measured',
                    'longer future moves test extrapolation beyond training coast speeds',
                    'fresh validation must run continuously with no pulse-boundary resets'])
    out=args.output/'model.json';out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2));print('SHA256',hashlib.sha256(out.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':main()
