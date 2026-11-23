#!/usr/bin/env python3
"""Compare completed independent PC tuning runs; read-only inputs, no alignment fit.
No refitting, waveform reset, timing shift, or matching gains after the fact.
Metrics use encoder counts, never the simulator's true velocity.
"""
import argparse
import hashlib
import json
from pathlib import Path
import numpy as np


def load(root):
    registered=json.loads((root/'registered.json').read_text())
    result=json.loads((root/'result.json').read_text())
    history=json.loads((root/'history.json').read_text())
    validations=[h for h in history if h['split']=='untouched_validation']
    if result['state']!='complete' or len(validations)!=1:raise ValueError('requires completed search and exactly one validation')
    trial=validations[0];case=root/f"trial-{trial['trial']:04d}"
    if trial['gains']!=result['best_gains']:raise ValueError('validation did not use frozen selected gains')
    if registered['backend']=='sim':
        data=np.loadtxt(case/'native-raw.csv',delimiter=',',skiprows=1)[::2]
        pos,effort,reference=data[:,0],data[:,1],data[:,2]
    else:
        # Resolve locally, not from historical absolute paths in the metadata.
        capture=root/'transport'/case.name
        report=json.loads((capture/'report.json').read_text())
        if not report['passed']:raise ValueError('hardware capture failed')
        for sid,filename in [('0','stream-0.i64le'),('2','stream-2.i32le')]:
            if hashlib.sha256((capture/filename).read_bytes()).hexdigest()!=report['streams'][sid]['sha256']:
                raise ValueError('hardware stream hash mismatch: '+sid)
        hello=json.loads((root/'transport/health.json').read_text())['hello']
        if hello['analog_position_servo']['source_sha256']!=registered['controller_identity']:
            raise ValueError('installed controller identity mismatch')
        encoder=np.fromfile(capture/'stream-0.i64le',dtype='<i8')
        telemetry=np.fromfile(capture/'stream-2.i32le',dtype='<i4').reshape(-1,2)
        if len(encoder)!=2*len(telemetry)-1:raise ValueError('unaligned hardware streams')
        pos=encoder[::2]-encoder[0];effort,reference=telemetry[:,0],telemetry[:,1]
    expected=np.r_[np.zeros(registered['pre_roll_ms']),np.repeat(registered['validation_references'],20),np.zeros(registered['post_roll_ms']+1)]
    np.testing.assert_array_equal(reference,expected)
    if len(pos)!=len(expected):raise ValueError('unexpected recording length')
    if hashlib.sha256((root/'model.json').read_bytes()).hexdigest()!=registered['frozen_model_sha256']:
        raise ValueError('frozen model bytes changed')
    return registered,result,trial,np.asarray(pos),np.asarray(effort),np.asarray(reference)


def step_metrics(position,reference,command_end):
    """Direct encoder observables, not a fitted damping model.
    2% settling: first sample after which ALL remaining samples in that hold
    stay inside +/-2% of the requested step. 1/e envelope decay is measured
    from first maximum overshoot to a persistent absolute-error band of peak/e.
    A ringing period is deliberately not inferred from a single overshoot.
    """
    changes=np.flatnonzero(np.diff(reference))+1;rows=[]
    for j,start in enumerate(changes):
        end=int(changes[j+1]) if j+1<len(changes) else command_end
        target=float(reference[start]);old=float(reference[start-1]);delta=target-old
        error=(position[start:end]-target)*np.sign(delta)
        progress=(position[start:end]-old)/delta
        def crossing(f):
            indices=np.flatnonzero(progress>=f);return int(indices[0]) if len(indices) else None
        def permanent_band(signal,band):
            outside=np.flatnonzero(np.abs(signal)>band)
            k=int(outside[-1]+1) if len(outside) else 0
            return k if k<len(signal) else None
        lo,hi=crossing(.1),crossing(.9)
        overshoot=max(0,float(np.max(error)));peak=int(np.argmax(error))
        decay=permanent_band(error[peak:],overshoot/np.e) if overshoot>=2 else None
        opposite=int(np.argmin(error[peak:]))+peak
        resolved=overshoot>=2 and error[opposite]<=-2
        rows.append(dict(step=j+1,target_counts=target,delta_counts=delta,
            overshoot_counts=overshoot,overshoot_percent=100*overshoot/abs(delta),
            rise_10_90_ms=hi-lo if lo is not None and hi is not None else None,
            settling_2_percent_ms=permanent_band(error,abs(delta)*.02),
            envelope_1e_decay_ms=decay,
            overshoot_to_undershoot_ms=opposite-peak if resolved else None,
            opposite_extremum_ratio=float(-error[opposite]/overshoot) if resolved else None,
            late100ms_error_counts=float(np.mean(position[end-100:end]-target)),
            ringing_period_ms=None,ringing_note='Not inferred: no robust multi-cycle estimate established.',
            slot_start_ms=int(start),slot_end_ms=end))
    return rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sim',type=Path,required=True);ap.add_argument('--hardware',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    sim=load(args.sim);hw=load(args.hardware)
    if sim[0]['backend']!='sim' or hw[0]['backend']!='hardware':raise ValueError('backend labels reversed')
    for key in ('source_sha256','controller_identity','frozen_model_sha256','seed_gains','references',
                'reference_period_ms','pre_roll_ms','post_roll_ms','rounds','score','validation_references'):
        if sim[0][key]!=hw[0][key]:raise ValueError('different tuning experiment: '+key)
    for key in ('search_policy','finalist_repeats','expanded_bounds','expanded_stages','validation_distance_counts'):
        if sim[0].get(key)!=hw[0].get(key):raise ValueError('different tuning policy: '+key)
    args.output.mkdir(parents=True,exist_ok=False)
    command_end=sim[0]['pre_roll_ms']+len(sim[0]['validation_references'])*20
    metrics={name:step_metrics(run[3],run[5],command_end) for name,run in [('sim',sim),('hardware',hw)]}
    for name,run in [('sim',sim),('hardware',hw)]:
        for step in metrics[name]:
            end=step['slot_end_ms'];step['late100ms_mean_effort_code']=float(np.mean(run[4][end-100:end]))
    summary=dict(schema='axis-pc-tuning-comparison-v1',hardware_comparison_done=True,
        frozen_model_sha256=sim[0]['frozen_model_sha256'],controller_identity=sim[0]['controller_identity'],
        report_source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        input_paths=dict(sim=str(args.sim),hardware=str(args.hardware)),
        input_registered_sha256={name:hashlib.sha256((root/'registered.json').read_bytes()).hexdigest()
            for name,root in [('sim',args.sim),('hardware',args.hardware)]},
        gains={name:run[1]['best_gains'] for name,run in [('sim',sim),('hardware',hw)]},
        validation={name:run[1]['validation_metrics'] for name,run in [('sim',sim),('hardware',hw)]},
        search_policy=sim[0].get('search_policy','legacy'),
        finalist_repeats=sim[0].get('finalist_repeats',0),
        bound_hits={name:run[1].get('bound_hits',[]) for name,run in [('sim',sim),('hardware',hw)]},
        steps=metrics,limits=[('Two-round factor-of-two coordinate search; selected position gain is at its bound.'
            if sim[0].get('search_policy','legacy')=='legacy' else
            'Expanded bounded search, 25% then 10% refinement; check bound_hits and finalist repetition spread.'),
            'Identical grid selections do not establish gain agreement to a few percent or a unique optimum.',
            'One fresh validation sequence per backend; repeated training finalists are not validation confidence intervals.',
            'Feedback can mask plant mismatch; not electrical/current-model validation.',
            'No robust multi-cycle ringing period established; envelope decay is a direct descriptive measurement.'])
    (args.output/'comparison.json').write_text(json.dumps(summary,indent=2,allow_nan=False)+'\n')
    (args.output/'axis_pc_compare.py').write_bytes(Path(__file__).read_bytes())
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig=plt.figure(figsize=(12,10));gs=fig.add_gridspec(3,2,height_ratios=(1.1,1,1))
    top=fig.add_subplot(gs[0,:]);zoom=fig.add_subplot(gs[1,0]);error_ax=fig.add_subplot(gs[1,1]);effort_ax=fig.add_subplot(gs[2,:])
    factor=360/4096;t=np.arange(len(sim[3]))/1000
    top.step(t,sim[5]*factor,where='post',color='0.5',lw=1,label='Requested position')
    start=metrics['sim'][0]['slot_start_ms'];end=start+350
    zoom.step((np.arange(start,end)-start),sim[5][start:end]*factor,where='post',color='0.5',lw=1,label='Request')
    for name,run,color,style in [('Hardware',hw,'#0072B2','-'),('Frozen model',sim,'#D55E00','--')]:
        position,effort,reference=run[3:]
        top.plot(t,position*factor,style,color=color,lw=1.6,label=name)
        zoom.plot(np.arange(end-start),position[start:end]*factor,style,color=color,lw=1.5,label=name)
        error_ax.plot(np.arange(end-start),(position-reference)[start:end]*factor,style,color=color,lw=1.5,label=name)
        effort_ax.plot(t,effort/32767*100,style,color=color,lw=1,label=name)
    amplitude=abs(metrics['sim'][0]['delta_counts']);band=amplitude*.02*factor
    error_ax.axhspan(-band,band,color='0.5',alpha=.15,label='2% band')
    top.set(xlabel='Device-relative capture time (s)',ylabel='Encoder position (degrees)',xlim=(0,command_end/1000+.1))
    zoom.set(xlabel='Time since first validation step (ms)',ylabel='Encoder position (degrees)',title=f'First +{amplitude*factor:g}° step: no fitted time shift')
    error_ax.set(xlabel='Time since first validation step (ms)',ylabel='Position error (degrees)',title='Tail residual: settling differs despite similar mean error',xlim=(40,300),ylim=(-3.5,3.5))
    effort_ax.set(xlabel='Device-relative capture time (s)',ylabel='Commanded effort (% FS)',xlim=(0,command_end/1000+.1))
    for ax in (top,zoom,error_ax,effort_ax):ax.grid(True,alpha=.25)
    top.legend(loc='lower left');error_ax.legend(loc='lower right');effort_ax.legend(loc='lower left')
    fig.suptitle('Same PC tuner, independent simulated and real feedback\nSM232AE/TQ10 • production Control.c • frozen empirical command-v2 plant',fontsize=14)
    labels=[]
    for name,run in [('Hardware',hw),('Model',sim)]:
        g=run[1]['best_gains'];labels.append(f"{name}: Ppos={g['position_kp_q8']}, Pv={g['velocity_kp_q8']}, Iv={g['velocity_ki_q8']}")
    caption=' | '.join(labels)+' (Q8.8).\n'
    caption+=f"First-step 2% settling: hardware {metrics['hardware'][0]['settling_2_percent_ms']} ms vs model {metrics['sim'][0]['settling_2_percent_ms']} ms. No plant refit or timing alignment."
    fig.text(.5,.015,caption,ha='center',fontsize=10)
    fig.tight_layout(rect=(0,.065,1,.94))
    for ext in ('png','svg'):fig.savefig(args.output/('same-pc-tuner.'+ext),dpi=140)
    # Presentation overview is exactly the top row of the diagnostic figure.
    # Keep the same recorded traces, axes, legend and schedule alignment.
    fig.canvas.draw()
    overview=top.get_tightbbox(fig.canvas.get_renderer()).transformed(fig.dpi_scale_trans.inverted()).expanded(1.02,1.08)
    fig.savefig(args.output/'same-pc-tuner-overview.png',dpi=140,bbox_inches=overview)
    plt.close(fig)
    print(json.dumps(dict(gains=summary['gains'],validation=summary['validation'],steps=metrics),indent=2))


if __name__=='__main__':main()
