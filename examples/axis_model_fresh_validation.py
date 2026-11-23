#!/usr/bin/env python3
"""Register prediction BEFORE START, then collect one genuinely new continuous run.

Four +/-15% / 30ms launches followed, 80ms after launch end, by opposing 5% /
10ms braking pulses. 2kHz capture. Not previously used for model fitting.
Dry-run by default. No post-capture parameter/timing adjustment is performed.
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
from axis_model_fit import Model,NAMES,load


def waveform():
    values=[0]*600
    for i,start in enumerate((20,160,300,440)):
        sign=(-1)**i
        values[start:start+3]=[sign*4915]*3
        values[start+11]=sign*-1638
    return values


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--model',type=Path,required=True);ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--port');ap.add_argument('--execute-powered',action='store_true');args=ap.parse_args()
    values=waveform()
    if not args.execute_powered:
        print(json.dumps(dict(commands=values,command_period_ticks=10000,command_ceiling_code=4915,
                             travel_limit_counts=40960,duration_s=6,post_roll_s=1),indent=2));return
    if not args.port:raise ValueError('--port required')
    t=recorder.Transport(args.port,args.output)
    try:
        hello=recorder.health(t)
        if hello['tick_hz']!=1000000:raise ValueError('requires 1MHz tick')
        model_bytes=args.model.read_bytes();config=json.loads(model_bytes)
        (args.output/'model.json').write_bytes(model_bytes)
        kernel=Path(__file__).with_name('axis_command_model.c')
        shutil.copy2(kernel,args.output/kernel.name)
        p=[config['parameters'][n] for n in NAMES]
        relative_ticks=np.arange(-100000,7000001,500,dtype=np.int64)
        indices=relative_ticks//10000;u=np.zeros(len(indices));valid=(indices>=0)&(indices<len(values))
        u[valid]=np.array(values)[indices[valid]]/32767
        predictor=Model(args.output/'kernel')
        x,v=predictor.simulate(u[None,:],[len(u)],.0005,p);x=x[0];v=v[0]
        prediction_path=args.output/'predicted-before-start.csv'
        with prediction_path.open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['relative_tick','command_fraction','position_rad','velocity_rad_s'])
            w.writerows(zip(relative_ticks,u,x,v))
        registration=dict(model_sha256=hashlib.sha256(model_bytes).hexdigest(),
            kernel_sha256=hashlib.sha256(kernel.read_bytes()).hexdigest(),
            prediction_sha256=hashlib.sha256(prediction_path.read_bytes()).hexdigest(),
            command_sha256=recorder.wire.command_sha256(values),commands=values,
            registered_unix_ns=time.time_ns(),
            initialization='one zero relative position/velocity/filter state before capture; no resets during run',
            adjustment_after_capture='none; measured initial encoder position only defines display origin',
            criteria=dict(per_maneuver_final_error='<= max(5 degrees, 10% measured displacement)',
                          per_maneuver_peak_speed_error='<=15% of measured peak speed'),
            peak_command_code=4915,travel_limit_counts=40960)
        recorder.save(args.output/'registered-before-start.json',registration)
        print('REGISTERED',registration['model_sha256'],registration['prediction_sha256'],flush=True)
        result=recorder.record(t,hello,'fresh-push-brake',[0],False,500,6,command_values=values,
                 command_ceiling_code=4915,post_roll_ticks=1000000,travel_limit_counts=40960)
        print('CAPTURE',json.dumps(result),flush=True)
        if not result['passed']:raise RuntimeError('capture failed; no validation claim')
        ticks,actual_u,angle,dt,plan,run=load(args.output/'fresh-push-brake')
        if not np.array_equal(ticks-run['command_start_tick'],relative_ticks):raise ValueError('timebase mismatch')
        if not np.array_equal(u,actual_u):raise ValueError('command mismatch')
        measured=angle-angle[0];tt=relative_ticks/1e6
        idx=np.arange(len(x));lo=np.maximum(0,idx-10);hi=np.minimum(len(x)-1,idx+10)
        measured_v=(measured[hi]-measured[lo])/((hi-lo)*dt)
        predicted_v=(x[hi]-x[lo])/((hi-lo)*dt)
        error=np.rad2deg(x-measured);legs=[]
        for i,start in enumerate((.2,1.6,3.,4.4)):
            stop=(1.6,3.,4.4,6.)[i]
            a=int(np.searchsorted(tt,start-.025));b=int(np.searchsorted(tt,stop-.1))
            obs=float(np.rad2deg(measured[b]-measured[a]));sim=float(np.rad2deg(x[b]-x[a]))
            peak=float(np.max(abs(measured_v[a:b])));pred_peak=float(np.max(abs(predicted_v[a:b])))
            legs.append(dict(maneuver=i+1,measured_net_deg=obs,predicted_net_deg=sim,
                final_displacement_error_deg=sim-obs,measured_peak_speed_rad_s=peak,predicted_peak_speed_rad_s=pred_peak,
                displacement_pass=bool(abs(sim-obs)<=max(5,.1*abs(obs))),
                peak_speed_pass=bool(abs(pred_peak-peak)<=.15*peak)))
        metrics=dict(model_sha256=registration['model_sha256'],capture_passed=True,
           full_trajectory_position_rmse_deg=float(np.sqrt(np.mean(error**2))),
           maximum_absolute_position_error_deg=float(np.max(abs(error))),final_position_error_deg=float(error[-1]),
           velocity_rmse_rad_s=float(np.sqrt(np.mean((predicted_v-measured_v)**2))),
           prediction_registered_before_start=True,fit_on_this_capture=False,continuous_state=True,
           final_200ms_range_counts=float(np.ptp(measured[-401:])*4096/(2*np.pi)),
           maneuvers=legs,meets_registered_accuracy_criteria=all(l['displacement_pass'] and l['peak_speed_pass'] for l in legs))
        recorder.save(args.output/'validation-metrics.json',metrics)
        with (args.output/'real-vs-sim.csv').open('w',newline='') as f:
            w=csv.writer(f);w.writerow(['time_s','command_fraction','measured_rad','simulated_rad','measured_velocity_rad_s','simulated_velocity_rad_s','position_error_deg'])
            w.writerows(zip(tt,u,measured,x,measured_v,predicted_v,error))
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig,axes=plt.subplots(4,1,figsize=(12,11),sharex=True)
        for ax,real,sim,label in ((axes[0],np.rad2deg(measured),np.rad2deg(x),'Position (deg)'),
                                 (axes[1],measured_v,predicted_v,'Velocity (rad/s)')):
            ax.plot(tt,real,color='black',label='Measured',linewidth=1.5)
            ax.plot(tt,sim,color='tab:orange',label='Frozen model',linewidth=1.2,linestyle='--')
            ax.set_ylabel(label);ax.legend(loc='best')
        axes[2].step(tt,u*100,where='post',color='tab:blue');axes[2].set_ylabel('Command (% FS)')
        axes[3].plot(tt,error,color='tab:purple');axes[3].axhline(0,color='gray',linewidth=.7)
        axes[3].set_ylabel('Sim - real (deg)');axes[3].set_xlabel('Time relative to command start (s)')
        for ax in axes:ax.grid(True,alpha=.3)
        fig.suptitle('Fresh push/brake validation — prediction frozen before START; no state resets or retuning')
        fig.tight_layout();fig.savefig(args.output/'real-vs-sim.png',dpi=150);fig.savefig(args.output/'real-vs-sim.svg');plt.close(fig)
        print('VALIDATION',json.dumps(metrics),flush=True)
    finally:t.close()


if __name__=='__main__':main()
