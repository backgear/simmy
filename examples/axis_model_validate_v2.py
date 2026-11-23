#!/usr/bin/env python3
"""Freeze two-model predictions BEFORE a new short/long/braking validation run.

No fitting in this tool. Entire 13.1s capture simulated continuously from one
rest state. 18% max launch, below tested 20%; peak nominal current 0.594A.
Dry-run by default. Ten-revolution guard commands zero, NOT active braking.
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
from axis_model_fit import Model as V1,NAMES as N1,load
from axis_model_fit_v2 import Model as V2,NAMES as N2


def plan(random_seed=None):
    if random_seed is not None:
        rng=np.random.default_rng(random_seed)
        moves=[]
        for i in range(6):
            if i%2==0:
                first=int(rng.choice([-1,1]));sign=first
            else:sign=-first
            width=int(rng.choice([1,2,3]))
            amplitude=float(rng.uniform(.03,min(.18,.0072/(width*.02))))
            move=dict(label=f'random {i+1}: {sign*amplitude*100:+.2f}% / {width*20}ms',
                      start=10+100*i,width=width,code=sign*int(32767*amplitude))
            room=.0072-amplitude*width*.02
            if rng.random()<.5 and room>=.0004:
                brake=float(rng.uniform(.02,min(.08,room/.02)))
                move.update(brake_gap=int(rng.integers(2,11)),brake_width=1,
                            brake_code=-sign*int(32767*brake))
                move['label']+=' + opposing pulse'
            moves.append(move)
    else:
        moves=[dict(label='short +10% 20ms',start=10,width=1,code=3276),
           dict(label='short -10% 20ms',start=110,width=1,code=-3276),
           dict(label='long +18% 40ms',start=210,width=2,code=5898),
           dict(label='long -18% 40ms',start=310,width=2,code=-5898),
           dict(label='long +12% 60ms / -6% brake',start=410,width=3,code=3932,brake_gap=6,brake_width=1,brake_code=-1966),
           dict(label='long -12% 60ms / +8% brake',start=510,width=3,code=-3932,brake_gap=10,brake_width=2,brake_code=2621)]
    values=[0]*600
    for m in moves:
        a=m['start'];values[a:a+m['width']]=[m['code']]*m['width']
        if 'brake_code' in m:
            b=a+m['width']+m['brake_gap'];values[b:b+m['brake_width']]=[m['brake_code']]*m['brake_width']
    return values,moves


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model-v1',type=Path,required=True);ap.add_argument('--model-v2',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);ap.add_argument('--port')
    ap.add_argument('--random-seed',type=int)
    ap.add_argument('--execute-powered',action='store_true');args=ap.parse_args()
    values,moves=plan(args.random_seed)
    if not args.execute_powered:
        print(json.dumps(dict(commands=values,moves=moves,duration_s=12,command_period_ticks=20000,
                             post_roll_s=1,command_ceiling_code=5898,travel_limit_counts=40960),indent=2));return
    if not args.port:raise ValueError('--port required')
    t=recorder.Transport(args.port,args.output)
    try:
        hello=recorder.health(t)
        if hello['tick_hz']!=1000000:raise ValueError('requires 1MHz tick')
        ticks_rel=np.arange(-100000,13000001,500,dtype=np.int64);tt=ticks_rel/1e6
        idx=ticks_rel//20000;u=np.zeros(len(idx));valid=(idx>=0)&(idx<len(values))
        u[valid]=np.array(values)[idx[valid]]/32767
        predictions={};registrations={}
        for name,path,klass,names,kernel in (
             ('v1',args.model_v1,V1,N1,'axis_command_model.c'),
             ('v2',args.model_v2,V2,N2,'axis_command_model_v2.c')):
            target=args.output/(name+'-model.json');target.write_bytes(path.read_bytes());cfg=json.loads(target.read_text())
            source=Path(__file__).with_name(kernel);shutil.copy2(source,args.output/kernel)
            model=klass(args.output/'kernel')
            x,v=model.simulate(u[None,:],[len(u)],.0005,[cfg['parameters'][n] for n in names])
            predictions[name]=(x[0],v[0])
            registrations[name]=dict(model_sha256=digest(target),kernel_sha256=digest(source))
        before=args.output/'predicted-before-start.csv'
        with before.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['relative_tick','command_fraction','v1_position_rad','v1_velocity_rad_s','v2_position_rad','v2_velocity_rad_s'])
            w.writerows(zip(ticks_rel,u,*predictions['v1'],*predictions['v2']))
        registration=dict(models=registrations,prediction_sha256=digest(before),
           command_sha256=recorder.wire.command_sha256(values),commands=values,moves=moves,
           random_seed=args.random_seed,
           random_envelope=('3..18%, 20/40/60ms, total absolute pulse area <=0.0072 FS*s per maneuver; paired balanced signs; optional opposing 20ms pulse'
                            if args.random_seed is not None else None),
           registered_unix_ns=time.time_ns(),script_sha256=digest(__file__),
           initialization='one zero relative position/velocity/filter state before capture; NO boundary resets',
           postcapture_adjustment='none; remove measured initial position once',
           criteria=dict(per_maneuver_displacement='error <= max(5 degrees, 10% measured move)',
                         per_maneuver_peak_speed='error <=15% measured peak'))
        recorder.save(args.output/'registered-before-start.json',registration)
        print('REGISTERED',json.dumps(registrations),registration['prediction_sha256'],flush=True)
        result=recorder.record(t,hello,'fresh-short-long-braking',[0],False,500,12,
             command_values=values,command_ceiling_code=5898,post_roll_ticks=1000000,travel_limit_counts=40960)
        print('CAPTURE',json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('capture failed; no validation claim')
        ticks,actual_u,angle,dt,manifest,run=load(args.output/'fresh-short-long-braking')
        if not np.array_equal(ticks-run['command_start_tick'],ticks_rel):raise ValueError('timebase mismatch')
        if not np.array_equal(u,actual_u):raise ValueError('command mismatch')
        measured=angle-angle[0]
        ii=np.arange(len(tt));lo=np.maximum(0,ii-10);hi=np.minimum(len(tt)-1,ii+10)
        velocity=lambda x:(x[hi]-x[lo])/((hi-lo)*dt)
        mv=velocity(measured);metrics=dict(capture_passed=True,continuous_state=True,
           fit_on_this_capture=False,prediction_registered_before_start=True,models={},
           final_200ms_range_counts=float(np.ptp(measured[-401:])*4096/(2*np.pi)))
        estimates={};errors={};moving=abs(mv)>1
        for name,(x,v) in predictions.items():
            pv=velocity(x);estimates[name]=pv;error=np.rad2deg(x-measured);errors[name]=error;legs=[]
            for j,m in enumerate(moves):
                start=m['start']*.02;stop=moves[j+1]['start']*.02 if j+1<len(moves) else 12
                a=int(np.searchsorted(tt,start-.025));b=int(np.searchsorted(tt,stop-.1))
                obs=float(np.rad2deg(measured[b]-measured[a]));sim=float(np.rad2deg(x[b]-x[a]))
                peak=float(np.max(abs(mv[a:b])));pp=float(np.max(abs(pv[a:b])))
                legs.append(dict(maneuver=j+1,label=m['label'],measured_net_deg=obs,predicted_net_deg=sim,
                    displacement_error_deg=sim-obs,measured_peak_speed_rad_s=peak,predicted_peak_speed_rad_s=pp,
                    displacement_pass=bool(abs(sim-obs)<=max(5,.1*abs(obs))),peak_speed_pass=bool(abs(pp-peak)<=.15*peak)))
            metrics['models'][name]=dict(model_sha256=registrations[name]['model_sha256'],
               position_rmse_deg=float(np.sqrt(np.mean(error**2))),max_position_error_deg=float(np.max(abs(error))),
               final_position_error_deg=float(error[-1]),velocity_rmse_rad_s=float(np.sqrt(np.mean((pv-mv)**2))),
               moving_velocity_rmse_rad_s=float(np.sqrt(np.mean((pv[moving]-mv[moving])**2))),
               maneuvers=legs,meets_registered_accuracy_criteria=all(l['displacement_pass'] and l['peak_speed_pass'] for l in legs))
        recorder.save(args.output/'validation-metrics.json',metrics)
        with (args.output/'real-vs-sim.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','command_fraction','measured_rad','measured_velocity_rad_s',
              'v1_rad','v1_velocity_rad_s','v2_rad','v2_velocity_rad_s','v1_error_deg','v2_error_deg'])
            w.writerows(zip(tt,u,measured,mv,predictions['v1'][0],estimates['v1'],predictions['v2'][0],estimates['v2'],errors['v1'],errors['v2']))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(4,1,figsize=(13,11),sharex=True)
        for axis,real,key,label in ((axes[0],np.rad2deg(measured),0,'Position (deg)'),(axes[1],mv,1,'Velocity (rad/s)')):
            axis.plot(tt,real,color='black',label='Measured',linewidth=1.4)
            for name,color in (('v1','tab:blue'),('v2','tab:orange')):
                y=np.rad2deg(predictions[name][0]) if key==0 else estimates[name]
                axis.plot(tt,y,'--',color=color,label=name,linewidth=1.1)
            axis.set_ylabel(label);axis.legend()
        axes[2].step(tt,u*100,where='post');axes[2].set_ylabel('Command (% FS)')
        for name,color in (('v1','tab:blue'),('v2','tab:orange')):axes[3].plot(tt,errors[name],color=color,label=name)
        axes[3].axhline(0,color='gray',linewidth=.7);axes[3].set_ylabel('Sim - real (deg)');axes[3].legend()
        axes[3].set_xlabel('Time from command start (s)')
        for a in axes:a.grid(True,alpha=.3)
        fig.suptitle(('New randomized validation' if args.random_seed is not None else 'New short/long/braking validation')
                     +' — both predictions frozen before START; no resets or retuning')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/('real-vs-sim-v2.'+ext),dpi=150)
        plt.close(fig)
        fig,axes=plt.subplots(2,3,figsize=(14,8))
        for j,(axis,m) in enumerate(zip(axes.flat,moves)):
            start=m['start']*.02;stop=moves[j+1]['start']*.02 if j+1<len(moves) else 12
            a=int(np.searchsorted(tt,start-.025));b=int(np.searchsorted(tt,stop-.1));sl=slice(a,b+1)
            axis.plot(tt[sl]-start,np.rad2deg(measured[sl]-measured[a]),color='black',label='Measured')
            for name,color in (('v1','tab:blue'),('v2','tab:orange')):
                x=predictions[name][0];axis.plot(tt[sl]-start,np.rad2deg(x[sl]-x[a]),'--',color=color,label=name)
            axis.set_title(m['label']);axis.grid(True,alpha=.3);axis.set_xlabel('Time from launch (s)');axis.set_ylabel('Local displayed angle (deg)')
        axes[0,0].legend();fig.suptitle('Per-maneuver display origins only — simulations remain continuous, not reset')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/('real-vs-sim-v2-maneuvers.'+ext),dpi=150)
        plt.close(fig)
        print('VALIDATION',json.dumps(metrics),flush=True)
    finally:t.close()


if __name__=='__main__':main()
