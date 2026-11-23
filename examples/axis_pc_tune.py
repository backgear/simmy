#!/usr/bin/env python3
"""ONE PC gain-search implementation, two feedback backends. Not a plant fitter.
Hardware needs firmware 0.2.0 live position-servo mode and --execute-powered.
Simulation uses frozen v2, actual snapshotted Control.c, and encoder-delta feedback.
No online identification, parameter fitting, timing alignment or torque replay.
"""
import argparse
import ctypes as ct
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import time
import numpy as np
from axis_servo_config import DEFAULT,FIELDS,validate_servo,workload
from axis_servo_native import NAMES
ROOT=Path(__file__).resolve().parents[1]
CORE=Path('characterization_rig/lib/axis_pid')

def save(p,v):
    temporary=p.with_name(p.name+'.tmp')
    temporary.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n');temporary.replace(p)
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def snapshot(out,model):
    if json.loads(model.read_text()).get('schema')!='axis-command-model-v2':
        raise ValueError('unsupported plant family; do not silently ignore additional dynamics')
    names=['examples/'+n for n in ('axis_pc_tune.py','axis_servo_config.py','axis_servo_native.py',
        'axis_pid_native.c','axis_command_step.h','axis_command_model_v2.c','axis_recorder.py','axis_wire.py','axis_motion_guard.py')]
    names += [str(CORE/n) for n in ('Control.c','Control.h','Output.h','axis_pid.c','axis_pid.h','LICENSE',
                                  'source_id.h','source_id.json','upstream.json')]
    names += ['characterization_rig/src/axis_hw.cpp','characterization_rig/src/protocol.cpp',
              'characterization_rig/src/main.cpp','characterization_rig/include/axis_hw.h',
              'characterization_rig/include/axis_config.h','characterization_rig/platformio.ini']
    hashes={}
    for name in names:
        dest=out/'source'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(ROOT/name,dest);hashes[name]=sha(dest)
    shutil.copy2(model,out/'model.json')
    ident=json.loads((out/'source'/CORE/'source_id.json').read_text())
    actual={n:sha(out/'source'/CORE/n) for n in ident['files']}
    if actual!=ident['files'] or hashlib.sha256(json.dumps(actual,sort_keys=True).encode()).hexdigest()!=ident['sha256']:
        raise ValueError('controller source identity is stale; regenerate identity and rebuild firmware')
    return hashes,ident['sha256']


class Config(ct.Structure):_fields_=[(n,ct.c_int32) for n in FIELDS]
class State(ct.Structure):_fields_=[(n,ct.c_double) for n in ('x','v','z')]


class SimBackend:
    def __init__(self,out):
        self.out=out;self.state=State();src=out/'source';core=src/CORE
        command=['cc','-std=c99','-O2','-shared','-fPIC','-Wall','-Wextra','-Werror',
            '-I',str(core),'-I',str(src/'examples'),str(src/'examples/axis_pid_native.c'),
            str(core/'axis_pid.c'),str(core/'Control.c'),'-lm','-o',str(out/'backend.so')]
        proc=subprocess.run(command,capture_output=True,text=True)
        (out/'build.log').write_text(proc.stdout+proc.stderr);save(out/'compile-command.json',command);proc.check_returncode()
        self.lib=ct.CDLL(str((out/'backend.so').resolve()))
        self.lib.axis_pid_native.argtypes=[ct.POINTER(State),ct.c_void_p,ct.POINTER(Config),ct.c_void_p,
            ct.c_int,ct.c_int,ct.c_int,ct.c_int,ct.c_void_p];self.lib.axis_pid_native.restype=ct.c_int
        model=json.loads((out/'model.json').read_text())
        if model['encoder_counts_per_rev']!=4096 or model['integration_step_s']!=.0005:raise ValueError('unsupported frozen plant units')
        self.params=np.array([model['parameters'][n] for n in NAMES],dtype=np.float64)
    def trial(self,directory,cfg,refs):
        # Physical state is NOT reset at trial boundaries; only the controller
        # gets the explicit fresh-start state also used by the MCU at START.
        before={n:getattr(self.state,n) for n in ('x','v','z')}
        refs=np.asarray(refs,dtype=np.int32);ticks=100+len(refs)*20+1000
        data=np.zeros((ticks*2+1,5));config=Config(**cfg)
        status=self.lib.axis_pid_native(ct.byref(self.state),self.params.ctypes.data,ct.byref(config),
            refs.ctypes.data,len(refs),20,100,1000,data.ctypes.data)
        save(directory/'plant-state.json',dict(before=before,after={n:getattr(self.state,n) for n in ('x','v','z')},fault_index=status-1 if status else None))
        if status:data=data[:max(0,status-1)]
        np.savetxt(directory/'native-raw.csv',data,delimiter=',',header='encoder_relative_count,effort_code,reference_count,truth_x_rad,truth_v_rad_s',comments='')
        if status:raise RuntimeError('simulated controller interlock; search stopped, no automatic retry')
        return data[::2,0],data[::2,1],data[::2,2],data[:,0]
    def close(self):pass


class HardwareBackend:
    def __init__(self,out,port,identity):
        from axis_motion_guard import require_motion_unpaused
        require_motion_unpaused()
        from axis_recorder import Transport,health
        self.t=Transport(port,out/'transport')
        try:
            self.hello=health(self.t)
            if self.hello.get('analog_position_servo',{}).get('source_sha256')!=identity:
                raise ValueError('installed live-controller source does not match archived backend; no motion attempted')
        except BaseException:self.t.close();raise
    def trial(self,directory,cfg,refs):
        from axis_recorder import record
        result=record(self.t,self.hello,directory.name,[0],False,500,len(refs)*.020,
            command_values=refs,post_roll_ticks=1000000,travel_limit_counts=4096,
            command_ceiling_code=cfg['torque_limit_code'],servo=cfg)
        case=self.t.directory/directory.name
        save(directory/'hardware-case.json',dict(path=str(case),report=result))
        if not result['passed']:raise RuntimeError('hardware capture failed; inspect preserved state, no automatic retry')
        encoder=np.fromfile(case/'stream-0.i64le',dtype='<i8');telemetry=np.fromfile(case/'stream-2.i32le',dtype='<i4').reshape(-1,2)
        if len(encoder)!=2*len(telemetry)-1:raise ValueError('encoder/effort stream alignment mismatch')
        encoder=encoder-encoder[0]
        return encoder[::2],telemetry[:,0],telemetry[:,1],encoder
    def close(self):self.t.close()


def score(position,effort,reference,raw_encoder,distance,limit):
    """Fixed sensor-only objective, identical on both backends. Units explicit."""
    if len(position)!=len(effort) or len(position)!=len(reference):raise ValueError('unaligned feedback')
    if np.ptp(raw_encoder[:200])>2 or np.ptp(raw_encoder[-400:])>4:
        raise RuntimeError('unsettled baseline/tail; search stopped rather than continuing from uncertain rest')
    # Use command interval, exclude the zero-effort post-roll from optimizer cost.
    pos=position[100:-1000];ref=reference[100:-1000];u=effort[100:-1000]
    error=pos-ref
    changes=np.r_[0,np.flatnonzero(np.diff(ref))+1,len(ref)]
    tails=np.concatenate([error[max(a,b-100):b] for a,b in zip(changes[:-1],changes[1:])])
    mae=float(np.mean(np.abs(error)));late=float(np.mean(np.abs(tails)))
    saturated=float(np.mean(np.abs(u)>=limit));excursion=int(np.ptp(raw_encoder))
    return dict(cost=mae/distance+.25*late/distance+.05*saturated,
        mean_absolute_error_counts=mae,late_error_counts=late,saturated_fraction=saturated,
        excursion_counts=excursion,motion_adequate=excursion>=distance//4,
        peak_effort_code=int(np.max(np.abs(effort))),tail_range_counts=int(np.ptp(raw_encoder[-400:])))


def search(evaluate,seed,rounds=2):
    """Bounded deterministic integer coordinate search. NO backend branching.
    Tests the incumbent then half/double in each coordinate; strict improvement
    only. No model-derived gain suggestions or automatic fault recovery.
    """
    current=dict(seed);best=evaluate(current)
    if not best['motion_adequate']:raise RuntimeError('pilot produced inadequate motion; refusing a long stationary campaign')
    for _ in range(rounds):
        changed=False
        for key,bounds in (('velocity_kp_q8',(8000,512000)),('velocity_ki_q8',(64,16384)),('position_kp_q8',(1,16))):
            center=current[key]
            for value in sorted({max(bounds[0],center//2),min(bounds[1],center*2)}-{center}):
                candidate=dict(current);candidate[key]=value;metrics=evaluate(candidate)
                if metrics['motion_adequate'] and metrics['cost']<best['cost']:
                    current,best=candidate,metrics;changed=True
        if not changed:break
    return current,best


EXPANDED_BOUNDS=(('velocity_kp_q8',(8000,512000)),('velocity_ki_q8',(64,16384)),('position_kp_q8',(1,32)))

def expanded_search(evaluate,seed):
    """Predeclared bounded expansion then +/-25%, +/-10% refinement.
    Keeps the old score and power limits. Position cap expands 16 -> 32 only;
    a winner on any bound is reported, not called an unconstrained optimum.
    """
    current=dict(seed);best=evaluate(current)
    if not best['motion_adequate']:raise RuntimeError('inadequate pilot motion; search stopped')
    for fraction,rounds in ((1.0,3),(.25,2),(.10,2)):
        for _ in range(rounds):
            changed=False
            for key,(lo,hi) in EXPANDED_BOUNDS:
                center=current[key]
                factors=(.5,2.) if fraction==1.0 else (1-fraction,1+fraction)
                candidates=sorted({min(hi,max(lo,round(center*f))) for f in factors}-{center})
                for value in candidates:
                    candidate=dict(current);candidate[key]=value;metrics=evaluate(candidate)
                    if metrics['motion_adequate'] and metrics['cost']<best['cost']:
                        current,best=candidate,metrics;changed=True
            if not changed:break
    return current,best


def repeat_finalists(evaluate,history,repeats):
    """Top three distinct candidates, actual interleaved repetitions, median rank.
    Fresh repeats alone determine the winner; initial noisy selection samples
    are not pooled into the repeated scores. No validation samples are used.
    """
    unique={}
    for entry in history:
        if entry['split']!='search':continue
        key=tuple(entry['gains'][f] for f in FIELDS)
        if key not in unique or entry['metrics']['cost']<unique[key]['metrics']['cost']:unique[key]=entry
    finalists=sorted(unique.values(),key=lambda e:e['metrics']['cost'])[:3]
    samples=[[] for _ in finalists]
    for repetition in range(repeats):
        for offset in range(len(finalists)):
            i=(offset+repetition)%len(finalists)
            samples[i].append(evaluate(dict(finalists[i]['gains']),role='finalist-repeat'))
    summary=[]
    for item,values in zip(finalists,samples):
        costs=[v['cost'] for v in values];median=float(np.median(costs))
        representative=min(values,key=lambda v:abs(v['cost']-median))
        summary.append(dict(gains=item['gains'],costs=costs,median_cost=median,
            min_cost=min(costs),max_cost=max(costs),representative_metrics=representative))
    winner=min(summary,key=lambda e:e['median_cost'])
    return dict(winner['gains']),winner['representative_metrics'],summary


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--backend',choices=('sim','hardware'),default='sim');p.add_argument('--port')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--model',type=Path,default=ROOT/'models/axis/sm232ae_tq10_command_v2.json')
    p.add_argument('--execute-powered',action='store_true');p.add_argument('--pilot-only',action='store_true')
    p.add_argument('--zero-only',action='store_true',help='pilot with zero position requests; tests controller/telemetry before moving')
    p.add_argument('--rounds',type=int,default=2)
    p.add_argument('--policy',choices=('legacy','expanded-v1'),default='legacy')
    p.add_argument('--finalist-repeats',type=int,default=0)
    args=p.parse_args()
    if args.backend=='hardware' and (not args.execute_powered or not args.port):p.error('hardware requires --port and --execute-powered')
    if not 1<=args.rounds<=8:p.error('rounds must be 1..8')
    if args.zero_only and not args.pilot_only:p.error('--zero-only requires --pilot-only')
    if args.finalist_repeats not in (0,3,5):p.error('finalist repeats must be 0, 3 or 5')
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=False)
    hashes,identity=snapshot(out,args.model);refs=[0]*50 if args.zero_only else workload()
    validation_distance=448 if args.policy=='expanded-v1' else 384
    validation_refs=workload(distance=validation_distance,hold_samples=45 if args.policy=='expanded-v1' else 35)
    seed=validate_servo(dict(DEFAULT))
    save(out/'registered.json',dict(backend=args.backend,source_sha256=hashes,controller_identity=identity,
        frozen_model_sha256=sha(out/'model.json'),seed_gains=seed,references=refs,reference_period_ms=20,
        pre_roll_ms=100,post_roll_ms=1000,rounds=args.rounds,pilot_only=args.pilot_only,
        zero_only=args.zero_only,validation_references=validation_refs,validation_distance_counts=validation_distance,
        search_policy=args.policy,finalist_repeats=args.finalist_repeats,
        expanded_bounds=dict(EXPANDED_BOUNDS) if args.policy=='expanded-v1' else None,
        expanded_stages=[dict(fraction=f,rounds=r) for f,r in ((1.,3),(.25,2),(.1,2))] if args.policy=='expanded-v1' else None,
        score='MAE/distance + 0.25*late100ms_MAE/distance + 0.05*saturated_fraction',
        note='Same PC search and sensor-only scoring, independent feedback. Model is never fitted or modified.'))
    backend=None;history=[]
    try:
        backend=SimBackend(out) if args.backend=='sim' else HardwareBackend(out,args.port,identity)
        def evaluate(cfg,validation=False,role='search'):
            if (out/'STOP').exists():raise RuntimeError('STOP requested at zero-command boundary')
            directory=out/f'trial-{len(history):04d}';directory.mkdir();save(directory/'config.json',cfg)
            selected_refs=validation_refs if validation else refs
            save(out/'progress.json',dict(state='running',completed_trials=len(history),active_trial=len(history),role=role))
            result=score(*backend.trial(directory,cfg,selected_refs),validation_distance if validation else 256,cfg['torque_limit_code'])
            entry=dict(trial=len(history),split='untouched_validation' if validation else 'search',role=role,gains=cfg,metrics=result);history.append(entry)
            save(out/'history.json',history);print(json.dumps(entry),flush=True)
            if not args.zero_only and not result['motion_adequate']:
                raise RuntimeError('inadequate motion; check drive/feedback before any further trial, no automatic retry')
            return result
        if args.pilot_only:
            best=seed;metrics=evaluate(seed)
            if not args.zero_only and not metrics['motion_adequate']:
                raise RuntimeError('moving pilot rejected: inadequate encoder motion; no search or automatic retry')
        else:
            best,metrics=expanded_search(evaluate,seed) if args.policy=='expanded-v1' else search(evaluate,seed,args.rounds)
            if args.finalist_repeats:
                best,metrics,repeated=repeat_finalists(evaluate,list(history),args.finalist_repeats)
                save(out/'finalist-repeats.json',repeated)
        bounds=dict(EXPANDED_BOUNDS) if args.policy=='expanded-v1' else dict(position_kp_q8=(1,16),velocity_kp_q8=(8000,512000),velocity_ki_q8=(64,16384))
        bound_hits=[k for k,(lo,hi) in bounds.items() if best[k] in (lo,hi)]
        save(out/'selected-gains.json',dict(gains=best,training_metrics=metrics,bound_hits=bound_hits))
        validation_metrics=None
        if not args.pilot_only:
            # Selection is frozen BEFORE the different-amplitude validation run;
            # no retuning or retries based on its result.
            validation_metrics=evaluate(best,validation=True)
        save(out/'result.json',dict(state='complete',backend=args.backend,best_gains=best,metrics=metrics,trials=len(history),
            validation_metrics=validation_metrics,bound_hits=bound_hits,search_policy=args.policy,hardware_comparison_done=False))
        save(out/'progress.json',dict(state='complete',completed_trials=len(history)))
    except BaseException as exc:
        save(out/'error.json',dict(error=repr(exc),completed_trials=len(history)))
        save(out/'progress.json',dict(state='stopped',error=repr(exc),completed_trials=len(history)));raise
    finally:
        if backend is not None:backend.close()


if __name__=='__main__':main()
