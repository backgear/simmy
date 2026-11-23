#!/usr/bin/env python3
"""Encoder-only targeted coast-history / low-command acquisition. Dry run default.
20% peak, <=.006 FS*s per maneuver, zero endpoints, 2s move spacing, 10rev
command-to-zero guard (NOT brake or drive disable). Stops suite on faults,
integrity errors or unsettled capture tails. Does not reflash or use ADC.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import time
import numpy as np
import axis_recorder as recorder
from axis_model_fit import load
from axis_model_fit_v2 import Model,NAMES


def plans(seed=23230):
    rng=np.random.default_rng(seed);groups={}
    groups['history']=[(amp,width,rep) for rep in range(3)
        for amp,width in ((.20,1),(.15,2),(.10,3),(.075,4))]
    groups['low_primary']=[(amp,width,rep) for rep in range(2)
        for amp in (.03,.04,.05,.06,.07) for width in (2,4)]
    groups['weak_hold']=[(amp,width,rep) for rep in range(2)
        for amp in (.005,.01,.015,.02) for width in (5,10)]
    out=[]
    for group,pairs in groups.items():
        order=rng.permutation(len(pairs));moves=[]
        for k in order:
            amp,width,rep=pairs[k];signs=[1,-1]
            if rng.integers(2):signs.reverse()
            for sign in signs:
                moves.append(dict(group=group,code=sign*int(32767*amp),width=width,repeat=rep+1,
                    label=f'{group}: {sign*amp*100:+g}% / {width*20}ms / repeat {rep+1}'))
        for batch in range(0,len(moves),6):
            mm=[];values=[0]*600
            for j,m in enumerate(moves[batch:batch+6]):
                m={**m,'start':10+100*j};mm.append(m)
                values[m['start']:m['start']+m['width']]=[m['code']]*m['width']
            out.append(dict(name=f'{group}-{batch//6+1:02d}',group=group,commands=values,moves=mm))
    return out


def indexed_plans(seed=23231):
    rng=np.random.default_rng(seed);levels=np.repeat([.01,.015,.02],6);rng.shuffle(levels)
    moves=[]
    for site,amp in enumerate(levels):
        sign=int(rng.choice([-1,1]));signs=[1,-1]
        if rng.integers(2):signs.reverse()
        moves.append(dict(group='index',code=sign*3276,width=1,site=site+1,
                          label=f'index site {site+1}: {sign*10:+g}% / 20ms'))
        for s in signs:
            moves.append(dict(group='indexed_hold',code=s*int(32767*amp),width=10,site=site+1,
                          label=f'site {site+1}: {s*amp*100:+g}% / 200ms'))
    out=[]
    for batch in range(0,len(moves),6):
        values=[0]*600;mm=[]
        for j,m in enumerate(moves[batch:batch+6]):
            m={**m,'start':10+100*j};mm.append(m)
            values[m['start']:m['start']+m['width']]=[m['code']]*m['width']
        out.append(dict(name=f'indexed_weak-{batch//6+1:02d}',group='indexed_weak',commands=values,moves=mm))
    return out


def save(path,obj):recorder.save(path,obj)
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--port')
    ap.add_argument('--model',type=Path,default=Path('models/axis/sm232ae_tq10_command_v2.json'))
    ap.add_argument('--execute-powered',action='store_true')
    ap.add_argument('--indexed-only',action='store_true');args=ap.parse_args()
    seed=23231 if args.indexed_only else 23230
    cases=indexed_plans(seed) if args.indexed_only else plans(seed)
    cfg=json.loads(args.model.read_text());p=[cfg['parameters'][n] for n in NAMES]
    if not args.execute_powered:
        print(json.dumps(dict(seed=seed,batches=len(cases),moves=sum(len(c['moves']) for c in cases),plans=cases),indent=2));return
    if not args.port:raise ValueError('--port required')
    t=recorder.Transport(args.port,args.output)
    try:
        hello=recorder.health(t)
        if hello['tick_hz']!=1000000:raise ValueError('requires 1MHz ticks')
        source=Path(__file__).with_name('axis_command_model_v2.c');shutil.copy2(source,args.output/source.name)
        shutil.copy2(args.model,args.output/'v2-model.json');kernel=Model(args.output/'kernel')
        registered=[];predictions={}
        for case in cases:
            ticks=np.arange(-100000,13000001,500,dtype=np.int64);idx=ticks//20000
            u=np.zeros(len(ticks));valid=(idx>=0)&(idx<600);u[valid]=np.array(case['commands'])[idx[valid]]/32767
            x,v=kernel.simulate(u[None,:],[len(u)],.0005,p)
            if np.max(abs(x))*4096/(2*np.pi)>8*4096:raise ValueError('predicted travel exceeds 8rev margin')
            if max(abs(m['code'])*m['width']*.02/32767 for m in case['moves'])>.00601:raise ValueError('pulse area')
            path=args.output/(case['name']+'-predicted-before-start.csv')
            with path.open('w',newline='') as f:
                w=csv.writer(f);w.writerow(['relative_tick','command_fraction','v2_position_rad','v2_velocity_rad_s'])
                w.writerows(zip(ticks,u,x[0],v[0]))
            predictions[case['name']]=x[0]
            registered.append({**case,'command_sha256':recorder.wire.command_sha256(case['commands']),
                                'prediction_file':path.name,'prediction_sha256':sha(path)})
        shutil.copy2(__file__,args.output/'acquisition-source.py')
        save(args.output/'registered-before-first-start.json',dict(seed=seed,registered_unix_ns=time.time_ns(),
            model_sha256=sha(args.model),kernel_sha256=sha(source),plans=registered,
            reserved_future_validation_seeds=[23240,23241],future_validation_not_yet_captured=True,
            adc=False,command_ceiling_code=6553,travel_limit_counts=40960,post_roll_ticks=1000000,
            initialization='one rest state per continuous capture; no maneuver resets',
            caveat='zero command does not remove jumper drive enable or provide active braking'))
        print('REGISTERED',len(cases),'batches',sum(len(c['moves']) for c in cases),'moves',flush=True)
        baseline=recorder.record(t,hello,'baseline-zero',[0],False,500,2,
            command_values=[0]*100,command_ceiling_code=6553,post_roll_ticks=1000000,travel_limit_counts=40960)
        if not baseline['passed']:raise RuntimeError('baseline capture integrity failed')
        ticks,u,a,dt,manifest,run=load(args.output/'baseline-zero')
        if np.ptp(a[-1001:])*4096/(2*np.pi)>2:raise RuntimeError('baseline not stationary')
        summaries=[]
        for case in cases:
            print('START',case['name'],flush=True)
            result=recorder.record(t,hello,case['name'],[0],False,500,12,
                 command_values=case['commands'],command_ceiling_code=6553,
                 post_roll_ticks=1000000,travel_limit_counts=40960)
            if not result['passed']:raise RuntimeError('integrity/fault: '+json.dumps(result))
            ticks,u,a,dt,manifest,run=load(args.output/case['name']);tt=(ticks-run['command_start_tick'])/1e6
            counts=a*4096/(2*np.pi);initial=counts[0];tail=float(np.ptp(counts[-401:]));rests=[]
            for m in case['moves']:
                start=m['start']*.02;window=counts[(tt>=start-.2)&(tt<=start-.02)]
                rests.append(float(np.ptp(window)))
            x=predictions[case['name']];row=dict(name=case['name'],group=case['group'],samples=len(a),
                final_200ms_range_counts=tail,pre_move_ranges_counts=rests,
                excursion_counts=float(np.max(abs(counts-initial))),
                v2_position_rmse_deg=float(np.rad2deg(np.sqrt(np.mean((x-(a-a[0]))**2)))))
            summaries.append(row);save(args.output/'acquisition-summary.json',summaries)
            print('COMPLETE',json.dumps(row),flush=True)
            if tail>4 or max(rests)>4:raise RuntimeError('unsettled tail/pre-move window; stop suite for review')
        save(args.output/'acquisition-complete.json',dict(passed=True,batches=len(cases),moves=sum(len(c['moves']) for c in cases)))
    finally:t.close()


if __name__=='__main__':main()
