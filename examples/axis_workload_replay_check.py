#!/usr/bin/env python3
"""Read-only continuous effort-replay backtests; NOT independent-controller tests.
Tests both actual-effort servo recordings and earlier uploaded-effort captures.
No serial, controller backend, trajectory generation, timing shifts or refitting.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from axis_fit_controller_workload import Batch,NAMES,load as load_servo
from axis_model_fit import load as load_effort

def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model',type=Path,action='append',required=True)
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('cases',nargs='+',type=Path);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);out=args.output
    source=out/'axis_command_model_v2.c';shutil.copy2(Path(__file__).with_name(source.name),source)
    shutil.copy2(Path(__file__),out/Path(__file__).name);model=Batch(source,out)
    models=[]
    for i,p in enumerate(args.model):
        cfg=json.loads(p.read_text())
        if cfg['kernel_sha256']!=sha(source):raise ValueError('model kernel identity mismatch')
        shutil.copy2(p,out/f'model-{i}.json');models.append((str(p),np.array([cfg['parameters'][n] for n in NAMES]),sha(p)))
    rows=[];squared_error=np.zeros(len(models));sample_count=0
    for j,case in enumerate(args.cases):
        manifest=json.loads((case/'plan.json').read_text())['manifest']
        if manifest['output_mode']=='analog_position_servo':u,x,_=load_servo(case)
        elif manifest['output_mode']=='analog_effort':
            _,u,angle,dt,_,_=load_effort(case)
            if dt!=.0005:raise ValueError('requires 2kHz')
            x=angle-angle[0]
        else:raise ValueError('unsupported recording type')
        u=np.ascontiguousarray(u[None,:],dtype=np.float64);lengths=np.array([u.shape[1]],dtype=np.int32)
        values=[];predictions=[]
        for name,p,digest in models:
            pred=model.simulate(u,lengths,p)[0];error=(pred-x)*180/np.pi
            values.append(dict(model=name,model_sha256=digest,position_rmse_deg=float(np.sqrt(np.mean(error**2))),
                final_position_error_deg=float(error[-1]),max_absolute_position_error_deg=float(max(abs(error)))))
            predictions.append(pred)
        squared_error+=np.sum(((np.array(predictions)-x)*180/np.pi)**2,axis=1);sample_count+=len(x)
        np.savez_compressed(out/f'case-{j:03d}.npz',effort=u[0],measured_rad=x,predicted_rad=np.array(predictions),dt=.0005)
        rows.append(dict(case=str(case),output_mode=manifest['output_mode'],models=values))
    result=dict(schema='axis-effort-replay-backtest-v1',rows=rows,kernel_sha256=sha(source),
        pooled_position_rmse_deg={name:float(v) for (name,_,_),v in zip(models,np.sqrt(squared_error/sample_count))},
        report_source_sha256=sha(out/Path(__file__).name),independent_validation=False,
        note='Seen development backtests; effort replay is not a fresh independently tuned controller comparison. No pulse-boundary resets or fitted time/position offsets.')
    (out/'summary.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(rows,indent=2))

if __name__=='__main__':main()
