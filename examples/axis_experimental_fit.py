#!/usr/bin/env python3
"""Offline candidate fits using EXACTLY the training sources in frozen v2.

No evaluation captures are loaded here. Candidate definitions/objective are saved
before fitting. Backtest selection is not independent validation.
"""
import argparse
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import time
import numpy as np
from scipy.optimize import least_squares,lsq_linear
from axis_model_fit import load

CANDIDATES=[('balanced_linear',0),('coast_first_linear',0),('quadratic_drag',1),
            ('speed_spline',2),('phase_torque',3),('bristle_memory',4)]


class Experimental:
    def __init__(self,build,source=None):
        build=Path(build);build.mkdir(parents=True,exist_ok=True)
        source=Path(source) if source is not None else Path(__file__).with_name('axis_experimental_models.c')
        version=hashlib.sha256(source.read_bytes()).hexdigest()[:12]
        lib=(build/('axis_experimental_'+version+'.so')).resolve()
        subprocess.run(['cc','-O3','-shared','-fPIC',str(source),
                        '-lm','-o',str(lib)],check=True)
        self.lib=ctypes.CDLL(str(lib));self.lib.axis_experimental.argtypes=[
            ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_int,ctypes.c_int,
            ctypes.c_double,ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p]
        self.lib.axis_experimental.restype=None

    def simulate(self,u,lengths,phases,p,kind,dt=.0005):
        u=np.ascontiguousarray(u,dtype=np.float64);lengths=np.ascontiguousarray(lengths,dtype=np.int32)
        phases=np.ascontiguousarray(phases,dtype=np.float64);p=np.ascontiguousarray(p,dtype=np.float64)
        x=np.zeros_like(u);v=np.zeros_like(u)
        self.lib.axis_experimental(u.ctypes.data,lengths.ctypes.data,phases.ctypes.data,
            len(lengths),u.shape[1],dt,p.ctypes.data,kind,x.ctypes.data,v.ctypes.data)
        return x,v


def derivative(x,lengths,dt=.0005):
    i=np.arange(x.shape[1])[None,:]
    lo=np.maximum(0,i-10);hi=np.minimum(np.array(lengths)[:,None]-1,i+10)
    lo=np.broadcast_to(lo,x.shape).clip(0,np.array(lengths)[:,None]-1)
    v=(np.take_along_axis(x,hi,1)-np.take_along_axis(x,lo,1))/np.maximum((hi-lo)*dt,dt)
    return np.where(i<np.array(lengths)[:,None],v,0)


def training(sources):
    episodes=[];coasts={1:[],-1:[]}
    for source in sources:
        ticks,u,angle,dt,plan,run=load(source)
        if dt!=.0005:raise ValueError('requires 2kHz')
        cmd=plan['commands'];cp=plan['manifest']['command_period_ticks'];origin=run['command_start_tick']
        active=[];i=0
        while i<len(cmd):
            if abs(cmd[i])<3276:i+=1;continue
            a=i;code=cmd[i]
            while i<len(cmd) and cmd[i]==code:i+=1
            active.append((origin+a*cp,origin+i*cp))
        for j,(a,b) in enumerate(active):
            end=min(b+1100000,active[j+1][0]-50000 if j+1<len(active) else ticks[-1])
            mask=(ticks>=a-50000)&(ticks<=end);xx=angle[mask];uu=u[mask]
            if np.ptp(xx[:80])>2*2*np.pi/4096:raise ValueError('nonquiet start')
            episodes.append((uu,xx-xx[0],xx[0]%(2*np.pi),source))
        # Pure zero-command training coast balances, not the +1% segments.
        ii=np.arange(len(angle));lo=np.maximum(0,ii-10);hi=np.minimum(len(angle)-1,ii+10)
        v=(angle[hi]-angle[lo])/((hi-lo)*dt)
        i=0
        while i<len(u):
            if u[i]!=0:i+=1;continue
            a=i
            while i<len(u) and u[i]==0:i+=1
            if a==0:continue
            windows=[]
            for j in range(a+40,i-40,20):
                k=j+40
                if v[j]*v[k]<=0 or min(abs(v[j]),abs(v[k]))<5:continue
                sign=1 if v[j]>0 else -1
                windows.append((sign,[.02,sign*(angle[k]-angle[j]),sign*(v[j]-v[k])]))
            for sign in (1,-1):
                w=[r for s,r in windows if s==sign]
                if w:coasts[sign].append(np.array(w))
    return episodes,coasts


def definitions(v2,coasts):
    p=v2['parameters'];base=[p['positive_gain_rad_s2_per_command'],p['negative_gain_rad_s2_per_command'],
       p['command_filter_tau_s'],p['command_deadband_fraction']]
    loss=[p['positive_constant_loss_rad_s2'],p['negative_constant_loss_rad_s2'],
          p['positive_viscous_s_inv'],p['negative_viscous_s_inv']]
    lower=[5000,5000,.00005,0];upper=[20000,20000,.01,.008]
    names=['Gpositive','Gnegative','tau_s','deadband_fraction']
    result={}
    for name,kind in CANDIDATES:
        fixed={};init=base+loss;lo=lower+[1,1,0,0];hi=upper+[500,300,10,10]
        labels=names+['Cpositive','Cnegative','Bpositive','Bnegative']
        if name=='coast_first_linear':
            for sign,cidx,bidx in ((1,4,6),(-1,5,7)):
                blocks=coasts[sign];data=np.concatenate([b/np.sqrt(len(b)) for b in blocks])
                x,y=data[:,:2],data[:,2];scales=np.linalg.norm(x,axis=0)
                fit=lsq_linear(x/scales,y,bounds=(0,np.inf));c,b=fit.x/scales
                fixed[cidx]=float(max(c,1));fixed[bidx]=float(b)
                init[cidx]=fixed[cidx];init[bidx]=fixed[bidx]
        elif kind==1:
            init=base+[260,80,.03,.02];lo=lower+[1,1,0,0];hi=upper+[500,300,.3,.3]
            labels=names+['Cpositive','Cnegative','Qpositive','Qnegative']
        elif kind==2:
            knots=np.array([0,5,15,30,45])
            init=base+list(loss[0]+loss[2]*knots)+list(loss[1]+loss[3]*knots)
            lo=lower+[1]*10;hi=upper+[700]*10
            labels=names+[f'Dpositive_at_{v}' for v in knots]+[f'Dnegative_at_{v}' for v in knots]
        elif kind==3:
            init+= [0]*4;lo+=[-150]*4;hi+=[150]*4
            labels+=['sin1','cos1','sin12','cos12']
        elif kind==4:
            c=(loss[0]+loss[1])/2;bias=(loss[1]-loss[0])/2
            init=base+[c,bias/c,loss[2],loss[3],8000,.02]
            lo=lower+[30,-.95,0,0,500,.001];hi=upper+[350,.95,10,10,30000,.03]
            labels=names+['bristle_C','bias_fraction','Bpositive','Bnegative','bristle_stiffness','bristle_damping_s']
        result[name]=dict(kind=kind,initial=init,lower=lo,upper=hi,labels=labels,fixed=fixed)
    return result


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--v2-model',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    v2=json.loads(args.v2_model.read_text());sources=v2['training_sources']
    episodes,coasts=training(sources)
    if len(episodes)!=v2['training_episodes']:raise ValueError('not exactly v2 episodes')
    n=len(episodes);stride=max(len(e[0]) for e in episodes)
    u=np.zeros((n,stride));y=np.zeros_like(u);mask=np.zeros_like(u,dtype=bool);lengths=[];phase=[]
    for i,(uu,xx,phi,source) in enumerate(episodes):
        u[i,:len(uu)]=uu;y[i,:len(xx)]=xx;mask[i,:len(xx):4]=True;lengths.append(len(xx));phase.append(phi)
    velocity=derivative(y,lengths)
    norms=np.sqrt(mask.sum(axis=1))[:,None]
    wx=1/np.maximum(np.ptp(y,axis=1),1)[:,None]/norms
    wv=1/np.maximum(np.max(abs(velocity),axis=1),5)[:,None]/norms
    specs=definitions(v2,coasts)
    experiment=dict(registered_unix_ns=time.time_ns(),training_episodes=n,training_sources=sources,
       v2_sha256=hashlib.sha256(args.v2_model.read_bytes()).hexdigest(),
       candidates=specs,objective='equal blocks normalized position and 10ms velocity; per-episode equal weighting; robust soft_l1 f_scale=.003',
       no_evaluation_data_loaded=True,backtest_warning='existing random traces are seen; not fresh validation')
    (args.output/'experiment-plan.json').write_text(json.dumps(experiment,indent=2)+'\n')
    model=Experimental(args.output);index=[]
    for name,spec in specs.items():
        initial=np.array(spec['initial']);lo=np.array(spec['lower']);hi=np.array(spec['upper'])
        fixed={int(k):v for k,v in spec['fixed'].items()};free=[i for i in range(len(initial)) if i not in fixed]
        def full(q):
            p=initial.copy();p[free]=q
            for k,v in fixed.items():p[k]=v
            return p
        def residual(q):
            x,v=model.simulate(u,lengths,phase,full(q),spec['kind'])
            vv=derivative(x,lengths)
            return np.concatenate((((x-y)*wx)[mask],((vv-velocity)*wv)[mask]))
        start=time.monotonic()
        fit=least_squares(residual,initial[free],bounds=(lo[free],hi[free]),x_scale='jac',
            loss='soft_l1',f_scale=.003,max_nfev=120,diff_step=1e-4,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        params=full(fit.x)
        record=dict(schema='axis-experimental-model-v1',name=name,kind=spec['kind'],parameter_order=spec['labels'],
            parameter_vector=list(map(float,params)),parameters=dict(zip(spec['labels'],map(float,params))),
            fixed_training_coast_parameters=fixed,training_sources=sources,training_episodes=n,
            initial_state='rest, measured initial encoder phase; bristle equilibrium at bias; no fitted per-episode state',
            objective=experiment['objective'],solver=dict(success=bool(fit.success),message=fit.message,
                nfev=fit.nfev,cost=float(fit.cost),seconds=time.monotonic()-start),
            kernel_sha256=hashlib.sha256(Path(__file__).with_name('axis_experimental_models.c').read_bytes()).hexdigest(),
            warning='experimental empirical model; no calibrated current; existing random backtests are not independent validation')
        path=args.output/(name+'.json');path.write_text(json.dumps(record,indent=2)+'\n')
        index.append(dict(name=name,file=path.name,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
        print(name,json.dumps(record['solver']),json.dumps(record['parameters']),flush=True)
    (args.output/'candidate-index.json').write_text(json.dumps(index,indent=2)+'\n')


if __name__=='__main__':main()
