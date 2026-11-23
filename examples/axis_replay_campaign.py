#!/usr/bin/env python3
"""Replay an external controller's recorded effort; fitter never chooses moves.

This is OPEN-LOOP replay, NOT a live analog-servo closed-loop comparison. 200ms
nonoverlapping source windows retain every 1kHz physical_torque sample. Select
only contiguous, enabled, fault-free windows within an existing bench envelope;
no rescaling, clipping, sign inversion or response/model-based selection.
A 1ms zero sample is added at each boundary for the sealed rig protocol.

Default is a plan-only run. --execute-powered authorizes motion. A checkpoint
allows --resume only from an IDLE device, with no retry/automatic clearing of a
faulted replay. STOP requests a zero-command stop at the next capture boundary.
The drive's jumper enable cannot be removed by this software.
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


def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract_windows(path,window=200,ceiling=6553,area_limit=.006):
    with Path(path).open(newline='') as f:rows=list(csv.DictReader(f))
    result=[];source_sha=sha(path)
    for start in range(0,len(rows)-window+1,window):
        block=rows[start:start+window]
        if any(r['enabled']!='1' or r['fault']!='none' for r in block):continue
        seq=[int(r['seq']) for r in block]
        if any(b!=a+1 for a,b in zip(seq,seq[1:])):continue
        values=[int(r['physical_torque']) for r in block];area=sum(map(abs,values))*.001/32767
        if max(map(abs,values))>ceiling or not 0<area<=area_limit:continue
        commands=[0]+values+[0];digest=recorder.wire.command_sha256(commands)
        result.append(dict(source=str(path),source_sha256=source_sha,csv_row_offset=start,
            first_source_seq=seq[0],last_source_seq=seq[-1],source_period_ticks=1000,
            source_window_samples=window,commands=commands,command_sha256=digest,
            absolute_command_area_fs_s=area,split='holdout' if int(digest[:8],16)%5==0 else 'train'))
    return result


def schedule(bank_size,count,seed):
    rng=np.random.default_rng(seed);out=[]
    while len(out)<count:out.extend(map(int,rng.permutation(bank_size)))
    return out[:count]


def checkpoint(root,obj):
    p=root/'progress.next.json';recorder.save(p,obj);p.replace(root/'progress.json')


def stationary(case):
    x=np.fromfile(case/'stream-0.i64le',dtype='<i8')
    return dict(samples=len(x),initial_count=int(x[0]),final_count=int(x[-1]),
        excursion_counts=int(np.max(abs(x-x[0]))),final_200ms_range_counts=int(np.ptp(x[-401:])))


def run(args):
    root=args.output
    if args.resume:
        plan=json.loads((root/'campaign-plan.json').read_text());progress=json.loads((root/'progress.json').read_text())
        if progress['state'] not in ('planned','boundary_stop','running') or 'active_replay' in progress:
            raise ValueError('resume requires a known zero boundary; reconcile interrupted captures/faults manually')
        bank=plan['bank'];order=plan['order'];completed=progress['completed']
        for item in bank:
            if recorder.wire.command_sha256(item['commands'])!=item['command_sha256']:raise ValueError('bank changed')
            if sha(root/item['archived_source'])!=item['source_sha256']:raise ValueError('archived input changed')
    else:
        if root.exists():raise FileExistsError(root)
        paths=sorted(args.source_root.rglob('*monitor.csv'));bank=[]
        for p in paths:bank.extend(extract_windows(p))
        unique={(b['source_sha256'],b['csv_row_offset']):b for b in bank}
        bank=[unique[k] for k in sorted(unique)]
        if not bank:raise ValueError('no eligible external workload windows')
        order=schedule(len(bank),args.count,args.seed);root.mkdir(parents=True)
        copies=root/'external-source';copies.mkdir();mapping={}
        for item in bank:
            original=Path(item['source'])
            if str(original) not in mapping:
                dest=copies/(item['source_sha256'][:16]+'-monitor.csv');shutil.copy2(original,dest)
                mapping[str(original)]=str(dest.relative_to(root))
                for name in ('entry-settings.json','events.jsonl','summary.json','results.json','winner.json'):
                    sibling=original.parent/name
                    if sibling.exists():shutil.copy2(sibling,copies/(item['source_sha256'][:16]+'-'+name))
            item['archived_source']=mapping[str(original)]
        shutil.copy2(__file__,root/'campaign-source.py')
        snapshots=root/'source-snapshots';snapshots.mkdir()
        for name in ('axis_recorder.py','axis_wire.py'):
            shutil.copy2(Path(__file__).with_name(name),snapshots/name)
        plan=dict(schema='independent-controller-replay-v1',created_unix_ns=time.time_ns(),
            count=args.count,seed=args.seed,bank=bank,order=order,source_csv_tick_hz=1000,
            encoder_hz=2000,command_ceiling_code=6553,area_limit_fs_s=.006,travel_limit_counts=40960,
            script_sha256=sha(__file__),source_order='source SHA-256 then row offset; independent of file names',
            post_roll_ticks=1500000,boundary_zero_samples_each_side=1,
            external_controller='recorded analog-servo-f103 tuning physical_torque',
            replay_not_live_closed_loop=True,selection_uses_no_model_or_measured_replay_response=True,
            split='command-hash grouped 20% holdout; all repeats of identical commands share split',
            unique_source_windows=len(bank),unique_command_traces=len(set(b['command_sha256'] for b in bank)),
            warning='1000 replay captures are NOT 1000 unique or independent excitation profiles; no active brake or enable removal')
        recorder.save(root/'campaign-plan.json',plan);completed=0
        checkpoint(root,dict(state='planned',completed=0,target=len(order),bank_windows=len(bank)))
    print('PLAN',len(order),'replays;',len(bank),'external windows;',len(set(b['command_sha256'] for b in bank)),'unique command traces',flush=True)
    if not args.execute_powered:return
    if not args.port:raise ValueError('--port required')
    segment=root/f'transport-from-{completed:06d}'
    t=recorder.Transport(args.port,segment)
    progress=dict(state='running',completed=completed,target=len(order),bank_windows=len(bank),segment=str(segment),
                  started_unix_ns=time.time_ns())
    try:
        hello=recorder.health(t)
        if hello['tick_hz']!=1000000:raise ValueError('requires 1MHz tick')
        baseline=recorder.record(t,hello,'baseline-zero',[0],False,500,1,
            command_values=[0]*100,post_roll_ticks=1500000,travel_limit_counts=40960,command_ceiling_code=6553)
        if not baseline['passed'] or stationary(segment/'baseline-zero')['final_200ms_range_counts']>2:
            raise RuntimeError('baseline integrity/stationarity failed')
        with (root/'completed.jsonl').open('a') as journal:
            while completed<len(order):
                if (root/'STOP').exists():
                    progress.update(state='boundary_stop',completed=completed);checkpoint(root,progress);print('STOPPED AT ZERO BOUNDARY',completed,flush=True);return
                index=order[completed];item=bank[index];name=f'replay-{completed+1:06d}'
                checkpoint(root,{**progress,'active_replay':completed+1,'completed':completed})
                print('START',completed+1,'bank',index,item['split'],flush=True)
                result=recorder.record(t,hello,name,[0],False,500,len(item['commands'])/1000,
                    command_values=item['commands'],command_ceiling_code=6553,
                    post_roll_ticks=1500000,travel_limit_counts=40960)
                if not result['passed']:raise RuntimeError('capture failed: '+json.dumps(result))
                motion=stationary(segment/name)
                if motion['final_200ms_range_counts']>4:raise RuntimeError('unsettled tail; review before continuing')
                row=dict(replay=completed+1,bank_index=index,split=item['split'],case=str(segment/name),
                    command_sha256=item['command_sha256'],completed_unix_ns=time.time_ns(),**motion)
                journal.write(json.dumps(row)+'\n');journal.flush();completed+=1
                progress.update(completed=completed,last_completed=row);checkpoint(root,progress)
                print('COMPLETE',completed,json.dumps(motion),flush=True)
        progress.update(state='complete');checkpoint(root,progress)
    except BaseException as exc:
        progress.update(state='stopped_error',completed=completed,error=f'{type(exc).__name__}: {exc}')
        checkpoint(root,progress);raise
    finally:t.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source-root',type=Path,default=Path('../analog-servo-f103/captures'))
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--port')
    ap.add_argument('--count',type=int,default=1000);ap.add_argument('--seed',type=int,default=23250)
    ap.add_argument('--execute-powered',action='store_true');ap.add_argument('--resume',action='store_true')
    args=ap.parse_args()
    if args.count<1:ap.error('--count must be positive')
    run(args)


if __name__=='__main__':main()
