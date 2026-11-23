#!/usr/bin/env python3
"""Batched +/-20% / 20ms bare-shaft coast trials. Dry-run unless explicit execute.

Four moves per 6s command plan, 1.4s apart, 2kHz encoder capture. Nominal 0.66A
from manual 1A/V and assumed unity 3.3V stage. ADC disabled. Ten-revolution
command-to-zero guard is NOT a brake or control of external jumper enable.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
import axis_recorder as recorder


def waveform(batch=1, coast_percent=0):
    values=[0]*600
    sign=1 if batch % 2 else -1
    for i,start in enumerate((20,160,300,440)):
        values[start:start+2]=[sign*(-1)**i*6553]*2
        if coast_percent:
            values[start+2:start+102]=[int(32767*coast_percent/100)]*100
    return values


def write_csv(path,rows):
    with Path(path).open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def extract(case):
    case=Path(case)
    capture=json.loads((case/'report.json').read_text())
    if not capture['passed'] or capture['state']!='COMPLETE':
        raise ValueError('invalid/incomplete capture')
    plan=json.loads((case/'plan.json').read_text())
    run=json.loads((case/'get_run.json').read_text())
    hz=json.loads((case.parent/'health.json').read_text())['hello']['tick_hz']
    desc=next(s for s in run['streams'] if s['stream_id']==0)
    counts=np.fromfile(case/'stream-0.i64le',dtype='<i8')
    if len(counts)!=desc['sample_count']:raise ValueError('sample count mismatch')
    dt=desc['period_ticks']/hz
    theta=counts*2*np.pi/4096
    ticks=desc['first_sample_tick']+np.arange(len(counts),dtype=np.int64)*desc['period_ticks']
    k=max(1,round(.005/dt));idx=np.arange(len(counts));lo=np.maximum(0,idx-k);hi=np.minimum(len(counts)-1,idx+k)
    v=(theta[hi]-theta[lo])/((hi-lo)*dt)
    cp=plan['manifest']['command_period_ticks'];origin=run['command_start_tick']
    pulses=[];i=0
    while i<len(plan['commands']):
        code=plan['commands'][i]
        if not code:i+=1;continue
        start=i
        while i<len(plan['commands']) and plan['commands'][i]==code:i+=1
        if abs(code)>=3276:
            pulses.append((origin+start*cp,origin+i*cp,code))
    metrics=[];windows=[]
    for leg,(start,end,code) in enumerate(pulses):
        sign=1 if code>0 else -1
        def values(a,b):return counts[(ticks>=a)&(ticks<=b)]
        pre=values(start-int(.075*hz),start-int(.025*hz))
        # Large trials allow >=1.38s zero between pulses; old 10% cases use .55s.
        endpoint=(.9,1.0) if abs(code)>3276 else (.45,.50)
        post=values(end+int(endpoint[0]*hz),end+int(endpoint[1]*hz))
        if len(pre)==0 or len(post)==0:raise ValueError('missing analysis window')
        a,b=float(np.median(pre)),float(np.median(post))
        peak_mask=(ticks>=start)&(ticks<=end+int(.1*hz))
        coast_index=min(len(plan['commands'])-1,int((end-origin+int(.5*hz))//cp))
        coast_code=plan['commands'][coast_index]
        metrics.append(dict(source=str(case),leg=leg,direction=sign,command_code=code,
          coast_command_fraction=coast_code/32767,
          command_fraction=code/32767,initial_count=a,final_count=b,displacement_deg=(b-a)*360/4096,
          initial_phase_deg=(a%4096)*360/4096,peak_speed_rad_s=float(np.max(sign*v[peak_mask])),
          pre_range_counts=int(np.ptp(pre)),post_range_counts=int(np.ptp(post)),
          stationary_windows=bool(np.ptp(pre)<=2 and np.ptp(post)<=2)))
        stop=min(pulses[leg+1][0]-int(.1*hz) if leg+1<len(pulses) else ticks[-1],end+int(1.1*hz))
        width=max(1,round(.02/dt));stride=max(1,round(.01/dt))
        for a in range(int(np.searchsorted(ticks,end+int(.02*hz))),int(np.searchsorted(ticks,stop))-width,stride):
            b=a+width
            if sign*v[a]<5 or sign*v[b]<5:continue
            ia=int((ticks[a]-origin)//cp);ib=int((ticks[b]-origin)//cp)
            ca=plan['commands'][ia] if 0<=ia<len(plan['commands']) else 0
            cb=plan['commands'][ib] if 0<=ib<len(plan['commands']) else 0
            if ca!=cb:continue
            windows.append(dict(source=str(case),leg=leg,direction=sign,
               command_fraction=abs(code)/32767,coast_command_fraction=ca/32767,tick_start=int(ticks[a]),tick_end=int(ticks[b]),
               speed_rad_s=float(sign*(v[a]+v[b])/2),
               deceleration_rad_s2=float(sign*(v[a]-v[b])/((b-a)*dt)),
               phase_deg=float((counts[(a+b)//2]%4096)*360/4096)))
    return metrics,windows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--port');ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--batches',type=int,default=6)
    ap.add_argument('--coast-percent',type=float,choices=(0,1),default=0)
    ap.add_argument('--execute-powered',action='store_true')
    args=ap.parse_args()
    if not 1<=args.batches<=8:raise ValueError('1..8 batches permitted')
    experiment=dict(command_period_ticks=10000,first_batch_commands=waveform(coast_percent=args.coast_percent),
       coast_command_percent=args.coast_percent,coast_command_duration_seconds=1,
       batches=args.batches,
       duration_seconds=6,post_roll_seconds=1,encoder_period_ticks=500,
       command_ceiling_code=6553,travel_limit_counts=40960,
       endpoint_window='900..1000ms after pulse end',input_kind='command, not measured effort')
    if not args.execute_powered:print(json.dumps(experiment,indent=2));return
    if not args.port:raise ValueError('--port required')
    t=recorder.Transport(args.port,args.output);all_metrics=[]
    try:
        h=recorder.health(t)
        if h['tick_hz']!=1000000:raise ValueError('requires 1MHz tick')
        recorder.save(args.output/'experiment.json',experiment)
        for batch in range(1,args.batches+1):
            name=f'batch-{batch:02d}'
            result=recorder.record(t,h,name,[0],False,500,6,command_values=waveform(batch,args.coast_percent),
              command_ceiling_code=6553,post_roll_ticks=1000000,travel_limit_counts=40960)
            print('CAPTURE',name,json.dumps(result),flush=True)
            if not result['passed']:raise RuntimeError('capture failed; no further batches')
            metrics,windows=extract(args.output/name);all_metrics.extend(metrics)
            write_csv(args.output/name/'pulse-metrics.csv',metrics)
            write_csv(args.output/'pulse-metrics.csv',all_metrics)
            print('MOVES',json.dumps(metrics),flush=True)
            if not all(m['stationary_windows'] for m in metrics):
                raise RuntimeError('nonstationary windows; no further batches')
    finally:t.close()


if __name__=='__main__':main()
