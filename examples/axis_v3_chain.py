#!/usr/bin/env python3
"""Read-only audit of friction-state carry across complete acquisition suites.

Between consecutive registered runs, assume the rig's documented idle output
of zero. Integrate the exact recorded clock gap, including 25us remainders.
Do not join separate suites or skip unobserved powered runs. One assumed-rest,
zero-load, fully-healed initial state per suite; no per-run position alignment.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
from axis_fit_controller_workload import NAMES, load, select_blocks, sha, save
from axis_model_fit import load as load_effort
from axis_v3_compare import native_parameters, csv_write, figure
from axis_model_v3 import Q
from axis_residual_diagnostics import verify_effort_capture

ROOT=Path(__file__).resolve().parents[1]


class Stateful:
    def __init__(self, source, out):
        out.mkdir(parents=True,exist_ok=True)
        library=(out/'stateful.so').resolve()
        subprocess.run(['cc','-O3','-shared','-fPIC',str(source),'-lm','-o',str(library)],check=True)
        self.lib=ct.CDLL(str(library))
        self.lib.axis_state_run.argtypes=[ct.c_int,ct.c_void_p,ct.c_int,ct.c_double,ct.c_void_p,ct.c_void_p,ct.c_void_p,ct.c_void_p]
        self.lib.axis_state_run.restype=ct.c_int
        self.lib.axis_state_gap.argtypes=[ct.c_int,ct.c_int64,ct.c_void_p,ct.c_void_p]
        self.lib.axis_state_gap.restype=ct.c_int

    @staticmethod
    def validate(family, params, state):
        lengths={0:8,1:12,2:12,3:14}
        if family not in lengths or np.shape(params)!=(lengths.get(family),):raise ValueError('unknown family/parameter length')
        if state.dtype!=np.float64 or state.shape!=(6,) or not state.flags.c_contiguous:
            raise ValueError('state must be a contiguous six-double array')
        if not np.all(np.isfinite(params)) or not np.all(np.isfinite(state)):raise ValueError('nonfinite input')

    def run(self, family, u, params, state, dt=.0005):
        self.validate(family,params,state)
        if not np.isfinite(dt) or dt<=0:raise ValueError('invalid timestep')
        u=np.ascontiguousarray(u,dtype=np.float64);p=np.ascontiguousarray(params,dtype=np.float64)
        if u.ndim!=1 or not np.all(np.isfinite(u)):raise ValueError('invalid effort array')
        x=np.zeros_like(u);v=np.zeros_like(u)
        if not self.lib.axis_state_run(family,u.ctypes.data,len(u),dt,p.ctypes.data,state.ctypes.data,x.ctypes.data,v.ctypes.data):
            raise ValueError('invalid stateful run')
        return x,v

    def gap(self, family, ticks, params, state):
        self.validate(family,params,state)
        if type(ticks) is not int or ticks<0 or ticks>2**63-1:raise ValueError('invalid gap ticks')
        p=np.ascontiguousarray(params,dtype=np.float64)
        if not self.lib.axis_state_gap(family,ticks,p.ctypes.data,state.ctypes.data):raise ValueError('invalid gap')


def initial_state(): return np.array([0.,0.,0.,0.,1.,0.])


def phase_gap(previous_tick,previous_count,tick,count):
    """Reject a clock/frame break before inventing a zero-input bridge."""
    if previous_tick is None:return None
    if tick<=previous_tick:raise ValueError('clock reset/overlapping captures inside suite')
    if abs(int(count)-int(previous_count))>1:raise ValueError('unverified encoder continuity at quiet boundary')
    return tick-previous_tick


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--fits',nargs='+',type=Path,required=True)
    ap.add_argument('--labels',nargs='+')
    ap.add_argument('--controller',nargs='*',type=Path,default=[])
    ap.add_argument('--effort-suite',nargs='*',type=Path,default=[])
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    if args.labels and (len(args.labels)!=len(args.fits) or len(set(args.labels))!=len(args.labels) or 'v2' in args.labels):
        raise ValueError('one distinct label per candidate required')
    out=args.output;out.mkdir(parents=True,exist_ok=False);source=out/'source';source.mkdir()
    for name in ['axis_v3_stateful.c','axis_command_step.h','axis_command_model_v3.c',
                 'axis_command_model_v3_slip.c','axis_command_model_v3_age.c']:
        shutil.copy2(ROOT/'examples'/name,source/name)
    engine=Stateful(source/'axis_v3_stateful.c',out/'native')
    seed=json.loads((args.fits[0]/'frozen-v2.json').read_text())
    models={'v2':(0,np.array([seed['parameters'][n] for n in NAMES]))};models_meta={}
    for i,fit in enumerate(args.fits):
        kernel,p,family=native_parameters(fit)
        if sha(source/kernel.name)!=sha(kernel):raise ValueError('adapter/kernel version mismatch')
        label=args.labels[i] if args.labels else 'v3'+chr(ord('a')+i)
        models[label]=({'speed-yield':1,'slip-yield':2,'age-yield':3}[family],p)
        models_meta[label]=dict(fit=str(fit),family=family,config_sha256=sha(fit/'candidate-model.json'))
    suites=[]
    for root in args.controller:
        training,dev,_=select_blocks(root);cases=training+dev
        ids=[int(p.name.split('-')[-1]) for p in cases]
        if ids!=list(range(ids[0],ids[-1]+1)):raise ValueError('cannot bridge excluded/unobserved controller runs')
        suites.append((root,cases,True,set(dev)))
    for root in args.effort_suite:
        reg=json.loads((root/'registered-before-first-start.json').read_text())
        suites.append((root,[root/p['name'] for p in reg['plans']],False,set()))
    metrics=[];inventory={};boundaries=[]
    for root,cases,servo,dev in suites:
        states={k:initial_state() for k in models};first_count=None;first_tick=None;last_tick=None;last_count=None
        for case in cases:
            run=json.loads((case/'get_run.json').read_text())
            if run.get('fault_reason') is not None or not run['capture_complete']:raise ValueError('invalid run')
            if servo:u,_,inv=load(case)
            else:
                _,u,_,dt,_,_=load_effort(case)
                if dt!=.0005:raise ValueError('2kHz required')
                inv=verify_effort_capture(case)
            count=np.fromfile(case/'stream-0.i64le',dtype='<i8')
            desc=next(s for s in run['streams'] if s['stream_id']==0)
            if desc['period_ticks']!=500 or u[0]!=0 or u[-1]!=0:raise ValueError('requires zero-output pre/postroll')
            tick=desc['first_sample_tick'];end=tick+(len(count)-1)*500
            if first_count is None:first_count=int(count[0]);first_tick=tick
            x=(count-first_count)*Q
            gap=phase_gap(last_tick,last_count,tick,int(count[0]))
            boundaries.append(dict(suite=str(root),source=str(case),gap_ticks=0 if gap is None else gap,
                first_count=int(count[0]),first_tick=tick,first_state_assumed=last_tick is None))
            predictions={}
            for name,(family,p) in models.items():
                if last_tick is not None:engine.gap(family,gap,p,states[name])
                pred,_=engine.run(family,u,p,states[name]);predictions[name]=pred
                err=pred-x;indices=np.arange(0,len(x),4)
                metrics.append(dict(suite=str(root),source=str(case),split='seen-development' if case in dev else 'training',model=name,
                    position_rmse_deg=float(np.sqrt(np.mean(err[indices]**2))*180/np.pi),
                    first_error_counts=float(err[0]/Q),last_error_counts=float(err[-1]/Q)))
            if servo and case==cases[-1]:
                telemetry=np.fromfile(case/'stream-2.i32le',dtype='<i4').reshape(-1,2)
                ref=(np.repeat(telemetry[:,1],2)[:len(u)]+int(count[0])-first_count)*Q
                t=np.arange(len(x))*.0005;changes=np.flatnonzero(np.diff(ref))+1
                windows=[('first step in last run',max(0,t[changes[0]]-.03),t[changes[0]]+.4),
                         ('last return in last run',max(0,t[changes[-1]]-.03),t[-1])]
                figure(out/(root.name+'-state-carry.png'),t,x,ref,u,predictions,windows,
                    root.name+' — state carried from first run; coordinates relative to suite origin',origin='suite')
                np.savez_compressed(out/(root.name+'-state-carry.npz'),t=t,real=x,reference=ref,effort=u,**predictions)
            inventory[str(case)]=inv;last_tick=end;last_count=int(count[-1])
    csv_write(out/'captures.csv',metrics);csv_write(out/'boundaries.csv',boundaries)
    save(out/'inventory.json',dict(models=models_meta,inputs=inventory,
        sources={p.name:sha(p) for p in sorted(source.iterdir())},adapter_sha256=sha(Path(__file__)),
        assumptions=['Zero input in documented idle gaps within a complete suite; no bridging separate suites.',
            'Only the first state per suite assumes rest/zero elastic load/fully healed strength.',
            'Reference origin uses measured first count at ARM, not a fitted model offset.'],
        independent_validation=False,no_hardware_interface=True))
    print('Completed state-carry audit:',len(suites),'suites,',len(boundaries),'runs')


if __name__=='__main__':main()
