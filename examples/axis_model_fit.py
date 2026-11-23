#!/usr/bin/env python3
"""Offline empirical current-command model fit. No hardware access.

Fits only explicitly supplied training bundles, never validation. Genuine quiet
intervals permit independently initialized training episodes. Validation must
simulate the entire capture continuously. Electrical constants remain separate;
fitted gain/deadband/filter are effective drive parameters, not calibrated amps.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import numpy as np
from scipy.optimize import least_squares

NAMES=['gain_rad_s2_per_command','bias_rad_s2','kinetic_rad_s2',
       'extra_static_rad_s2','stribeck_speed_rad_s','viscous_s_inv',
       'command_filter_tau_s','command_deadband_fraction']


class Model:
    def __init__(self, build):
        build=Path(build);build.mkdir(parents=True,exist_ok=True)
        source=Path(__file__).with_name('axis_command_model.c')
        library=(build/'axis_command_model.so').resolve()
        subprocess.run(['cc','-O3','-shared','-fPIC',str(source),'-lm','-o',str(library)],check=True)
        self.lib=ctypes.CDLL(str(library))
        self.lib.axis_model.argtypes=[ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,
                                     ctypes.c_int,ctypes.c_double,ctypes.c_void_p,
                                     ctypes.c_void_p,ctypes.c_void_p]
        self.lib.axis_model.restype=None

    def simulate(self,u,lengths,dt,p):
        u=np.ascontiguousarray(u,dtype=np.float64)
        lengths=np.ascontiguousarray(lengths,dtype=np.int32)
        p=np.ascontiguousarray(p,dtype=np.float64)
        x=np.zeros_like(u);v=np.zeros_like(u)
        self.lib.axis_model(u.ctypes.data,lengths.ctypes.data,len(lengths),u.shape[1],dt,
                            p.ctypes.data,x.ctypes.data,v.ctypes.data)
        return x,v


def load(case):
    case=Path(case)
    report=json.loads((case/'report.json').read_text())
    if not report['passed'] or report['state']!='COMPLETE':raise ValueError('invalid capture')
    plan=json.loads((case/'plan.json').read_text());run=json.loads((case/'get_run.json').read_text())
    hz=json.loads((case.parent/'health.json').read_text())['hello']['tick_hz']
    d=next(s for s in run['streams'] if s['stream_id']==0)
    count=np.fromfile(case/'stream-0.i64le',dtype='<i8')
    if len(count)!=d['sample_count']:raise ValueError('bad sample count')
    ticks=d['first_sample_tick']+np.arange(len(count),dtype=np.int64)*d['period_ticks']
    cp=plan['manifest']['command_period_ticks'];cmd=np.array(plan['commands'])/32767
    index=(ticks-run['command_start_tick'])//cp
    u=np.zeros(len(count));valid=(index>=0)&(index<len(cmd));u[valid]=cmd[index[valid]]
    return ticks,u,count*2*np.pi/4096,d['period_ticks']/hz,plan,run


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('training_bundles',nargs='+',type=Path)
    ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    episodes=[];sources=[]
    for bundle in args.training_bundles:
        if 'validation' in bundle.name:raise ValueError('validation bundle is not training')
        for case in sorted(bundle.glob('batch-*')):
            ticks,u,angle,dt,plan,run=load(case)
            if dt!=.0005:raise ValueError('training requires 2kHz')
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
                if np.ptp(xx[:80])>2*2*np.pi/4096:raise ValueError('nonquiet episode start')
                episodes.append((uu,xx-xx[0]));sources.append(str(case))
    stride=max(len(u) for u,x in episodes);n=len(episodes)
    uu=np.zeros((n,stride));yy=np.zeros_like(uu);mask=np.zeros_like(uu,dtype=bool)
    lengths=[];weights=[]
    for i,(u,x) in enumerate(episodes):
        uu[i,:len(u)]=u;yy[i,:len(x)]=x;mask[i,:len(x):4]=True
        lengths.append(len(u));weights.append(1/max(np.ptp(x),1.)/np.sqrt(np.sum(mask[i])))
    weights=np.array(weights)[:,None]
    model=Model(args.output)
    def residual(p):
        pred,_=model.simulate(uu,lengths,.0005,p)
        return ((pred-yy)*weights)[mask]
    initial=[11500,-90,160,40,8,.7,.001,.0005]
    lower=[5000,-250,20,0,1,0,.0001,0]
    upper=[20000,250,400,400,25,10,.015,.008]
    fit=least_squares(residual,initial,bounds=(lower,upper),x_scale='jac',
                      loss='soft_l1',f_scale=.003,max_nfev=120,ftol=1e-7,xtol=1e-7,gtol=1e-7)
    params=dict(zip(NAMES,[float(v) for v in fit.x]))
    result=dict(schema='axis-command-model-v1',status='frozen provisional empirical model; fresh validation pending',
      model='filtered deadbanded command + constant bias - Stribeck/Coulomb/viscous loss, with stiction',
      units='mechanical radians, seconds; normalized uploaded command, NOT measured current',
      encoder_counts_per_rev=4096,integration_step_s=.0005,parameters=params,
      training_episodes=n,training_sources=sorted(set(sources)),
      initialization='one quiet initial state per training episode; no per-sample resets',
      solver=dict(success=bool(fit.success),message=fit.message,nfev=fit.nfev,cost=float(fit.cost)),
      limitations=['no cogging/rotor-angle dependence','no hysteresis beyond stiction',
                   'effective command scale/filter/deadband not calibrated electronics',
                   'not a three-phase voltage/current simulation','no fit on reserved or fresh validation'])
    path=args.output/'model.json';path.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))
    print('SHA256',hashlib.sha256(path.read_bytes()).hexdigest(),flush=True)


if __name__=='__main__':main()
