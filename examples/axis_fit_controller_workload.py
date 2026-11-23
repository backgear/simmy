#!/usr/bin/env python3
"""READ-ONLY plant fitting from completed controller recordings, never motion.
No serial/recorder/controller backend imports or motion interface. Uses ACTUAL
recorded effort, not position references or controller gains. Original v2 equations
unchanged; command-filter tau/deadband fixed to prior seed for identifiability.
Later chronological gain-search blocks select among training-only multistarts;
finalist repetitions and control-validation recordings are never opened.
All current data are seen development evidence, NOT fresh physical validation.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from scipy.optimize import least_squares

NAMES=('positive_gain_rad_s2_per_command','negative_gain_rad_s2_per_command',
       'positive_constant_loss_rad_s2','negative_constant_loss_rad_s2',
       'positive_viscous_s_inv','negative_viscous_s_inv','command_filter_tau_s','command_deadband_fraction')
ROOT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def save(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')

def select_blocks(root):
    history=json.loads((root/'history.json').read_text())
    eligible=[e for e in history if e['split']=='search' and e.get('role','search')!='finalist-repeat']
    if len(eligible)<5:raise ValueError('requires at least five normal search trials per source')
    boundary=int(len(eligible)*.8)
    training=[root/'transport'/f"trial-{e['trial']:04d}" for e in eligible[:boundary]]
    development=[root/'transport'/f"trial-{e['trial']:04d}" for e in eligible[boundary:]]
    excluded=[dict(trial=e['trial'],reason=e['split'] if e['split']!='search' else 'finalist-repeat')
              for e in history if e not in eligible]
    return training,development,excluded


def load(case):
    report=json.loads((case/'report.json').read_text());run=json.loads((case/'get_run.json').read_text())
    plan=json.loads((case/'plan.json').read_text());manifest=plan['manifest']
    if not report['passed'] or report['state']!='COMPLETE':raise ValueError('invalid capture')
    if manifest['output_mode']!='analog_position_servo' or manifest['analog_enabled']:
        raise ValueError('requires actual servo effort telemetry, not ADC or reference-as-effort')
    if manifest['encoder_sample_period_ticks']!=500 or manifest['analog_sample_period_ticks']!=1000:
        raise ValueError('requires 2kHz encoder / 1kHz effort')
    hello=json.loads((case.parent/'health.json').read_text())['hello']
    if hello['tick_hz']!=1000000:raise ValueError('unsupported device clock')
    descriptors={str(d['stream_id']):d for d in run['streams']};inventory={}
    for sid,name in [('0','stream-0.i64le'),('2','stream-2.i32le')]:
        digest=sha(case/name)
        if digest!=report['streams'][sid]['sha256'] or digest!=descriptors[sid]['sha256']:
            raise ValueError('raw stream hash mismatch: '+name)
        inventory[name]=dict(sha256=digest,bytes=(case/name).stat().st_size)
    if descriptors['0']['period_ticks']!=500 or descriptors['2']['period_ticks']!=1000:
        raise ValueError('stream periods disagree with 2kHz/1kHz manifest')
    if descriptors['0']['first_sample_tick']!=descriptors['2']['first_sample_tick']:
        raise ValueError('unaligned acquisition origins; no timing-shift fit allowed')
    count=np.fromfile(case/'stream-0.i64le',dtype='<i8')
    telemetry=np.fromfile(case/'stream-2.i32le',dtype='<i4').reshape(-1,2)
    if len(count)!=descriptors['0']['sample_count'] or len(telemetry)!=descriptors['2']['sample_count']:
        raise ValueError('raw lengths disagree with stream descriptors')
    if len(count)!=2*len(telemetry)-1:raise ValueError('sample alignment mismatch')
    if np.ptp(count[:200])>2 or np.ptp(count[-400:])>4:raise ValueError('episode not quiet at baseline/tail')
    # telemetry column 1 is a position request, NEVER a force sample.
    effort=np.repeat(telemetry[:,0],2)[:len(count)]/32767.0
    angle=(count-count[0])*2*np.pi/4096
    return effort,angle,inventory


class Batch:
    def __init__(self,source,out):
        self.command=['cc','-O3','-shared','-fPIC',str(source),'-lm','-o',str(out/'kernel.so')]
        subprocess.run(self.command,check=True);self.lib=ct.CDLL(str((out/'kernel.so').resolve()))
        self.lib.axis_model.argtypes=[ct.c_void_p,ct.c_void_p,ct.c_int,ct.c_int,ct.c_double,ct.c_void_p,ct.c_void_p,ct.c_void_p]
        self.lib.axis_model.restype=None
    def simulate(self,u,lengths,p):
        p=np.ascontiguousarray(p,dtype=np.float64);x=np.zeros_like(u);v=np.zeros_like(u)
        self.lib.axis_model(u.ctypes.data,lengths.ctypes.data,len(lengths),u.shape[1],.0005,p.ctypes.data,x.ctypes.data,v.ctypes.data)
        return x


def arrays(episodes):
    lengths=np.array([len(e[0]) for e in episodes],dtype=np.int32);n=max(lengths)
    u=np.zeros((len(episodes),n));x=np.zeros_like(u);mask=np.zeros_like(u,dtype=bool)
    for i,(effort,angle,_) in enumerate(episodes):
        u[i,:len(effort)]=effort;x[i,:len(angle)]=angle;mask[i,:len(angle):4]=True
    weights=1/np.sqrt(mask.sum(axis=1))[:,None]
    return u,x,lengths,mask,weights


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recordings',type=Path,nargs='+');ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--seed-model',type=Path,default=ROOT/'models/axis/sm232ae_tq10_command_v2.json')
    ap.add_argument('--max-nfev',type=int,default=100);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);out=args.output
    seed_bytes=args.seed_model.read_bytes();seed=json.loads(seed_bytes)
    initial=np.array([seed['parameters'][k] for k in NAMES]);(out/'seed-model.json').write_bytes(seed_bytes)
    sources=[];development=[];excluded={}
    for root in args.recordings:
        train,dev,skip=select_blocks(root);sources+=train;development+=dev;excluded[str(root)]=skip
    if len(set(sources+development))!=len(sources+development):raise ValueError('duplicate recording inputs')
    training=[load(p) for p in sources];dev=[load(p) for p in development]
    inventory={str(p):e[2] for p,e in zip(sources+development,training+dev)}
    kernel=out/'axis_command_model_v2.c';shutil.copy2(ROOT/'examples/axis_command_model_v2.c',kernel)
    shutil.copy2(Path(__file__),out/'axis_fit_controller_workload.py')
    lower=np.array([3000,3000,0,0,0,0.]);upper=np.array([40000,40000,1500,1500,30,30.])
    starts=[initial[:6],initial[:6]*[1,1,2,3,.5,.5],initial[:6]*[1.1,1.1,1.5,1.5,1,1],initial[:6]*[.8,.8,3,3,2,2]]
    starts=[np.clip(p,lower+1e-7,upper-1e-7) for p in starts]
    save(out/'registered.json',dict(training_sources=list(map(str,sources)),development_sources=list(map(str,development)),
        excluded_sources=excluded,input_inventory=inventory,seed_model_sha256=hashlib.sha256(seed_bytes).hexdigest(),
        kernel_sha256=sha(kernel),fitter_sha256=sha(out/'axis_fit_controller_workload.py'),
        lower=lower.tolist(),upper=upper.tolist(),starts=[p.tolist() for p in starts],
        fixed_parameters={k:float(initial[i]) for i,k in enumerate(NAMES) if i>=6},
        max_nfev=args.max_nfev,objective='equal-case normalized position residual, one sample per 2ms; adaptive initial-median soft-L1 scale',
        split='first 80% normal search trials per run: coefficient training; later 20%: seen chronological development selection',
        no_motion_interface=True,independent_validation_done=False))
    model=Batch(kernel,out);u,x,lengths,mask,weights=arrays(training)
    du,dx,dl,dm,dw=arrays(dev)
    def parameters(free):return np.r_[free,initial[6:]]
    def residual(free):return ((model.simulate(u,lengths,parameters(free))-x)*weights)[mask]
    scale=max(float(np.median(np.abs(residual(initial[:6])))),1e-5)
    def rmse(uu,xx,ll,mm,p):return float(np.sqrt(np.mean((model.simulate(uu,ll,p)[mm]-xx[mm])**2))*180/np.pi)
    baseline=dict(training_position_rmse_deg=rmse(u,x,lengths,mask,initial),development_position_rmse_deg=rmse(du,dx,dl,dm,initial))
    save(out/'baseline.json',baseline);results=[]
    for i,start in enumerate(starts):
        fit=least_squares(residual,start,bounds=(lower,upper),x_scale='jac',loss='soft_l1',f_scale=scale,
            max_nfev=args.max_nfev,ftol=1e-7,xtol=1e-7,gtol=1e-7)
        p=parameters(fit.x);row=dict(start=i,parameters=dict(zip(NAMES,map(float,p))),solver_success=bool(fit.success),
            message=str(fit.message),nfev=fit.nfev,cost=float(fit.cost),
            training_position_rmse_deg=rmse(u,x,lengths,mask,p),development_position_rmse_deg=rmse(du,dx,dl,dm,p))
        results.append(row);save(out/'multistart-results.json',results);print(json.dumps(row),flush=True)
    chosen=min(results,key=lambda r:r['development_position_rmse_deg'])
    result=dict(seed);result.update(status='offline controller-workload candidate; NOT independently validated',
        parameters=chosen['parameters'],training_episodes=len(sources),training_sources=list(map(str,sources)),
        development_sources=list(map(str,development)),kernel_sha256=sha(kernel),solver=chosen,
        fixed_filter_deadband_from_seed=True,training_input='recorded actual effort code, not position requests',
        independent_validation_done=False,
        limitations=seed['limitations']+['Fit/selection use seen controller-workload data; fresh independent acquisition required.',
            'Filter/deadband fixed from seed; six effective directional dynamics parameters fitted.',
            'No calibrated current, torque, rotor-angle/history state, fitted timing shifts or initial-condition nuisance parameters.'])
    save(out/'candidate-model.json',result)
    save(out/'summary.json',dict(baseline=baseline,chosen=chosen,training_cases=len(sources),development_cases=len(development),
        candidate_sha256=sha(out/'candidate-model.json'),independent_validation_done=False,adaptive_f_scale=scale,
        native_compile_command=model.command))
    print(json.dumps(json.loads((out/'summary.json').read_text()),indent=2))

if __name__=='__main__':main()
