#!/usr/bin/env python3
"""Preview a finite ramp with the unchanged production controller, simulation only.

Both backends use the same fixed gains and 1kHz encoder-delta feedback. This is
independent closed-loop simulation, not matched-effort validation. No real trace
is fabricated from the incompatible legacy ramp. The proposal is not an ARM plan.
"""
import argparse
import ctypes as ct
import json
from pathlib import Path
import shutil
import subprocess
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from axis_fit_controller_workload import NAMES, sha, save
from axis_servo_config import FIELDS, validate_servo
from axis_v3_chain import Stateful, initial_state
from axis_v3_compare import native_parameters
from axis_model_v3 import Q
ROOT=Path(__file__).resolve().parents[1]


def ramp_reference(distance=128,sign=1):
    """600 finite 1ms position requests; <=1 count change per request."""
    if type(distance) is not int or not 1<=distance<=128 or type(sign) is not int or sign not in (-1,1):
        raise ValueError('bounded ramp requires integer distance 1..128 and sign +/-1')
    return sign*np.array([0]*50+list(range(1,distance+1))+[distance]*180+
        list(range(distance-1,-1,-1))+[0]*(370-2*distance),dtype=np.int32)


class Controller:
    class Config(ct.Structure):_fields_=[(name,ct.c_int32) for name in FIELDS]
    def __init__(self,source,out,config):
        validate_servo(config);self.config=self.Config(*(config[n] for n in FIELDS))
        library=(out/'controller.so').resolve()
        subprocess.run(['cc','-O3','-shared','-fPIC','-I'+str(source),str(source/'axis_pid.c'),
            str(source/'Control.c'),'-o',str(library)],check=True)
        self.lib=ct.CDLL(str(library))
        self.lib.axis_pid_init.argtypes=[ct.POINTER(self.Config),ct.c_int32];self.lib.axis_pid_init.restype=ct.c_bool
        self.lib.axis_pid_tick.argtypes=[ct.c_int32,ct.c_int32,ct.POINTER(ct.c_int32)];self.lib.axis_pid_tick.restype=ct.c_bool
    def init(self,position):
        if not self.lib.axis_pid_init(ct.byref(self.config),position):raise ValueError('controller initialization failed')
    def tick(self,reference,position):
        effort=ct.c_int32()
        if not self.lib.axis_pid_tick(reference,position,ct.byref(effort)):raise ValueError('latched native controller failure; no automatic restart')
        return effort.value


def main():
    ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--fit',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    out=args.output;out.mkdir(parents=True,exist_ok=False);source=out/'source';source.mkdir();core=out/'controller';core.mkdir()
    for name in ['axis_v3_stateful.c','axis_command_step.h','axis_command_model_v3.c','axis_command_model_v3_slip.c','axis_command_model_v3_age.c']:
        shutil.copy2(ROOT/'examples'/name,source/name)
    croot=ROOT/'characterization_rig/lib/axis_pid';identity=json.loads((croot/'source_id.json').read_text())
    for name,digest in identity['files'].items():
        if sha(croot/name)!=digest:raise ValueError('controller source identity mismatch')
        shutil.copy2(croot/name,core/name)
    shutil.copy2(croot/'LICENSE',core/'LICENSE');shutil.copy2(croot/'source_id.json',core/'source_id.json')
    config=dict(zip(FIELDS,(26,120000,15488,3276,1024,8)))
    controller=Controller(core,out,config);engine=Stateful(source/'axis_v3_stateful.c',out/'native')
    kernel,params,family=native_parameters(args.fit)
    if sha(source/kernel.name)!=sha(kernel):raise ValueError('adapter/kernel version mismatch')
    seed=json.loads((args.fit/'frozen-v2.json').read_text());p=np.array([seed['parameters'][n] for n in NAMES])
    models={'v2':(0,p),'v3 experimental':({'speed-yield':1,'slip-yield':2,'age-yield':3}[family],params)}
    requests=ramp_reference();pre=300;post=1000;reference=np.r_[np.zeros(pre,dtype=np.int32),requests,np.zeros(post,dtype=np.int32)]
    traces={};fig,axes=plt.subplots(4,1,figsize=(14,11),sharex=True,constrained_layout=True)
    t=np.arange(len(reference)+1)*.001
    for label,(code,p) in models.items():
        state=initial_state();controller.init(0);x=[];velocity=[];effort=[]
        for i,ref in enumerate(reference):
            x.append(state[0]);velocity.append(state[1])
            u=controller.tick(int(ref),int(np.rint(state[0]/Q))) if pre<=i<pre+len(requests) else 0
            effort.append(u);engine.run(code,np.full(3,u/32767.),p,state)
        x=np.r_[x,state[0]];velocity=np.r_[velocity,state[1]];effort=np.r_[effort,0]
        traces[label.replace(' ','_')+'_position']=x;traces[label.replace(' ','_')+'_effort']=effort
        color='tab:blue' if label=='v2' else 'tab:green'
        axes[0].plot(t,x/Q,label=label,color=color);axes[1].plot(t,velocity,color=color)
        axes[2].plot(t,x/Q-np.r_[reference,0],color=color);axes[3].step(t,effort/32767*100,where='post',color=color)
    axes[0].plot(t,np.r_[reference,0],'--',color='0.45',label='common reference')
    for ax,label in zip(axes,['position [count]','true model velocity [rad/s]','reference error [count]','command [% full scale]']):
        ax.set_ylabel(label);ax.grid(alpha=.25);ax.axvspan((pre+len(requests))*.001,t[-1],color='0.8',alpha=.25)
    axes[0].legend();axes[-1].set_xlabel('time [s]; shaded postroll forces zero effort, NOT braking')
    fig.suptitle('SIMULATION ONLY — same fixed production controller, independent feedback\nNo real ramp capture; initial rest/zero elastic load/fully healed strength are assumptions')
    fig.savefig(out/'ramp-preview.png',dpi=160);plt.close(fig)
    np.savez_compressed(out/'traces.npz',t=t,reference=np.r_[reference,0],**traces)
    save(out/'proposal.json',dict(status='OFFLINE PROPOSAL ONLY; NOT AN EXECUTABLE ARM PLAN; NO POWERED AUTHORIZATION',
        reference_period_ticks=1000,position_reference_counts=requests.tolist(),pre_roll_ms=pre,post_roll_ms=post,
        controller=config,controller_source_identity=identity,model_config_sha256=sha(args.fit/'candidate-model.json'),
        sources={p.name:sha(p) for p in sorted(source.iterdir())},script_sha256=sha(Path(__file__)),
        requires=['Fresh explicit powered-test authorization and reconciled drive fault/status.',
            'Travel, inhibit, output-path and capture compatibility checks; zero effort does not isolate the jumper-enabled drive.'],
        comparison='Independent feedback with identical fixed gains, not independent tuning or same-effort replay.'))
    print(out/'ramp-preview.png')


if __name__=='__main__':main()
