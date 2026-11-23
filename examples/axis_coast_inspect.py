#!/usr/bin/env python3
"""Encoder-only offline terminal-coast/stop-phase inspection; no model fitting."""
import argparse
import json
from pathlib import Path
import numpy as np
from axis_model_fit import load


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--v2-model',type=Path,required=True);ap.add_argument('--bundle',type=Path,required=True)
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    sources=json.loads(args.v2_model.read_text())['training_sources'];initial=[];final=[]
    quantum=2*np.pi/4096
    for source in sources:
        ticks,u,angle,dt,plan,run=load(source);cmd=plan['commands'];cp=plan['manifest']['command_period_ticks']
        active=[];i=0
        while i<len(cmd):
            if abs(cmd[i])<3276:i+=1;continue
            a=i;code=cmd[i]
            while i<len(cmd) and cmd[i]==code:i+=1
            active.append(run['command_start_tick']+a*cp)
        for j,start in enumerate(active):
            end=active[j+1]-100000 if j+1<len(active) else ticks[-1]-100000
            aa=angle[(ticks>=start-75000)&(ticks<=start-25000)]
            bb=angle[(ticks>=end-50000)&(ticks<=end)]
            if len(aa) and len(bb) and np.ptp(bb)<=2*quantum:
                initial.append(float(np.median(aa)));final.append(float(np.median(bb)))
    harmonics=[dict(harmonic=n,initial_concentration=float(abs(np.mean(np.exp(1j*n*np.array(initial))))),
                    final_concentration=float(abs(np.mean(np.exp(1j*n*np.array(final)))))) for n in range(1,25)]
    result=dict(training_stops=len(final),harmonics=harmonics,
        warning='Starts are not uniform and stops are not independent; concentrations are exploratory, not proof of cogging order.',
        count_quantum_rad=quantum,velocity_quantum_10ms_rad_s=quantum/.01)
    (args.output/'training-stop-phase.json').write_text(json.dumps(result,indent=2)+'\n')
    ticks,u,angle,dt,plan,run=load(args.bundle/'fresh-short-long-braking')
    if dt!=.0005:raise ValueError('requires 2kHz')
    velocity={};ii=np.arange(len(angle))
    for window in (.005,.01,.02):
        half=round(window/2/dt);lo=np.maximum(0,ii-half);hi=np.minimum(len(angle)-1,ii+half)
        velocity[window]=(angle[hi]-angle[lo])/((hi-lo)*dt)
    selected=[];i=0
    while i<len(u) and len(selected)<4:
        if u[i]!=0:i+=1;continue
        start=i
        while i<len(u) and u[i]==0:i+=1
        if start==0 or i-start<800 or abs(velocity[.01][start+40])<5:continue
        if np.ptp(angle[i-200:i])>2*quantum:continue
        changes=np.flatnonzero(abs(np.diff(angle[start:i]))>.4*quantum)
        if len(changes):selected.append((start,i,start+int(changes[-1])+1))
    observations=[]
    for start,end,stop in selected:
        sign=1 if velocity[.01][start+40]>0 else -1
        extreme=np.max(angle[start:end]) if sign>0 else np.min(angle[start:end])
        observations.append(dict(initial_velocity_rad_s=float(velocity[.01][start+40]),
            reverse_excursion_counts=float(sign*(extreme-angle[end-1])/quantum),
            reverse_excursion_deg=float(sign*np.rad2deg(extreme-angle[end-1])),
            last_update_tick=int(ticks[stop])))
    (args.output/'terminal-observations.json').write_text(json.dumps(observations,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(2,len(selected),figsize=(16,7),squeeze=False)
    for j,(start,end,stop) in enumerate(selected):
        a=max(start,stop-500);b=min(end,stop+200);tt=(ticks[a:b]-ticks[stop])/1e6
        counts=np.rint((angle[a:b]-angle[end-1])/quantum)
        axes[0,j].step(tt,counts,where='post',color='black');axes[0,j].set_ylabel('Counts relative to final')
        axes[0,j].set_title(f'Zero-command coast {j+1}')
        for window,v in velocity.items():axes[1,j].plot(tt,v[a:b],label=f'{window*1000:g}ms estimate')
        axes[1,j].set_ylabel('Velocity (rad/s)');axes[1,j].set_xlabel('Time relative to last encoder edge (s)')
        axes[1,j].legend(fontsize=8)
        for ax in axes[:,j]:ax.grid(True,alpha=.3);ax.axvline(0,color='gray',lw=.7)
    fig.suptitle('Terminal coast: raw encoder steps and estimator-window sensitivity (random seed 23210)')
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(args.output/('terminal-coasts.'+ext),dpi=150)
    plt.close(fig)
    print(json.dumps(dict(training_stops=len(final),selected_terminal_coasts=len(selected),
                          harmonic12=harmonics[11],terminal_observations=observations),indent=2))


if __name__=='__main__':main()
