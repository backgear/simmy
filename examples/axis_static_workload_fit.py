#!/usr/bin/env python3
"""Fit one effective breakaway threshold from existing controller recordings.
Read-only files, no motion interface. Frozen v2 moving/filter dynamics unchanged.
Train on chronological prefixes; evaluate later blocks and ALL supplied old
coast captures without using them to select the threshold. All data are seen
candidate-development evidence, not fresh independent physical validation.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from axis_fit_controller_workload import Batch,NAMES,arrays,load,select_blocks
from axis_model_fit import load as load_effort

def save(p,v):p.write_text(json.dumps(v,indent=2,allow_nan=False)+'\n')
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('recordings',nargs='+',type=Path)
    ap.add_argument('--coast',action='append',type=Path,required=True)
    ap.add_argument('--kind',choices=['static-only','stribeck'],default='static-only')
    ap.add_argument('--frozen-model',required=True,type=Path)
    ap.add_argument('--output',required=True,type=Path);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False);out=args.output
    seed=json.loads(args.frozen_model.read_text());p=np.array([seed['parameters'][n] for n in NAMES])
    if seed['schema']!='axis-command-model-v2':raise ValueError('requires frozen v2 seed')
    train=[];dev=[];excluded={}
    for root in args.recordings:
        a,b,c=select_blocks(root);train+=a;dev+=b;excluded[str(root)]=c
    training=[load(c) for c in train];development=[load(c) for c in dev];coasts=[]
    for case in args.coast:
        _,u,x,dt,_,_=load_effort(case)
        if dt!=.0005:raise ValueError('requires 2kHz coasts')
        coasts.append((u,x-x[0],{}))
    grid=([(s,) for s in range(0,401,5)] if args.kind=='static-only' else
          [(s,v) for s in range(0,401,25) for v in [.1,.25,.5,1,2,4,8]])
    kernel=('axis_command_model_static_v1.c' if args.kind=='static-only' else 'axis_command_model_stribeck_v1.c')
    extra_names=['extra_static_loss_rad_s2']+([] if args.kind=='static-only' else ['stribeck_velocity_scale_rad_s'])
    source=out/'source';source.mkdir()
    for name in ['axis_static_workload_fit.py','axis_fit_controller_workload.py','axis_model_fit.py',
                 'axis_command_model_v2.c','axis_command_model_static_v1.c','axis_command_model_stribeck_v1.c']:
        shutil.copy2(Path(__file__).with_name(name),source/name)
    shutil.copy2(args.frozen_model,out/'seed-model.json')
    if sha(source/'axis_command_model_v2.c')!=seed['kernel_sha256']:raise ValueError('seed kernel identity mismatch')
    save(out/'registered.json',dict(training_sources=list(map(str,train)),development_sources=list(map(str,dev)),
        coast_sources=list(map(str,args.coast)),excluded=excluded,kind=args.kind,
        extra_parameter_grid=[dict(zip(extra_names,row)) for row in grid],
        all_other_parameters_fixed=seed['parameters'],selection='minimum equal-case training position MSE only',
        independent_validation=False,source_sha256={c.name:sha(c) for c in source.iterdir()},seed_sha256=sha(args.frozen_model),
        input_raw_hashes={str(c):e[2] for c,e in zip(train+dev,training+development)}))
    (out/'baseline').mkdir();(out/'static').mkdir()
    baseline=Batch(source/'axis_command_model_v2.c',out/'baseline')
    model=Batch(source/kernel,out/'static')
    datasets={name:arrays(e) for name,e in [('training',training),('development',development),('older_coasts',coasts)]}
    def metrics(native,params,data):
        u,x,lengths,mask,weights=data;error=native.simulate(u,lengths,params)-x
        return dict(position_rmse_deg=float(np.sqrt(np.mean(error[mask]**2))*180/np.pi),
                    equal_case_mse_rad2=float(np.sum(((error*weights)[mask])**2)/len(lengths)))
    reference={name:metrics(baseline,p,d) for name,d in datasets.items()}
    rows=[]
    for extra in grid:
        values={name:metrics(model,np.r_[p,list(extra)],d) for name,d in datasets.items()}
        row=dict(zip(extra_names,extra));row['metrics']=values;rows.append(row)
        save(out/'grid.json',rows)
    chosen=min(rows,key=lambda r:r['metrics']['training']['equal_case_mse_rad2'])
    result=dict(schema='axis-command-model-'+('static-v1' if args.kind=='static-only' else 'stribeck-v1'),
        status='experimental offline candidate; not independently validated or promoted',
        parameters=dict(seed['parameters'],**{n:chosen[n] for n in extra_names}),
        encoder_counts_per_rev=4096,integration_step_s=.0005,kernel_sha256=sha(source/kernel),
        original_v2_sha256=sha(args.frozen_model),selection=chosen,
        limitations=['Effective restart threshold, not identified intrinsic friction or measured torque.',
                     'No angle/history/elastic friction state; moving dynamics fixed to frozen v2.',
                     'Training and all evaluations are seen development data; fresh physical validation required.',
                     'Additional parameters are not supported by the current eight-parameter independent tuner adapter.'])
    save(out/'candidate-model.json',result)
    save(out/'summary.json',dict(baseline=reference,chosen=chosen,training_cases=len(train),development_cases=len(dev),
        older_coast_cases=len(coasts),independent_validation=False,candidate_sha256=sha(out/'candidate-model.json')))
    print(json.dumps(json.loads((out/'summary.json').read_text()),indent=2))

if __name__=='__main__':main()
