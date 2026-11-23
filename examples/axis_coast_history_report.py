#!/usr/bin/env python3
"""Offline report of targeted acquisitions; no fitting or hardware commands.
Coast acceleration windows are reduced to one mean per move/speed bin before
reporting between-move scatter. Uploaded command is NOT measured current.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from axis_model_fit import load


def savecsv(path,rows):
    with path.open('w',newline='') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)


def velocity(x,dt):
    i=np.arange(len(x));lo=np.maximum(i-10,0);hi=np.minimum(i+10,len(x)-1)
    return (x[hi]-x[lo])/((hi-lo)*dt)


def coast_windows(ticks,x,v,dt,start,end,code,family,source,move):
    a=np.searchsorted(ticks,start+20000);b=np.searchsorted(ticks,end-10000);rows=[]
    sign=1 if code>0 else -1
    for j in range(a,b-40,40):
        k=j+40
        if min(sign*v[j],sign*v[k])<5:continue
        speed=sign*.5*(v[j]+v[k]);bucket=int(speed//5)*5
        if bucket<5 or bucket>=45:continue
        rows.append(dict(source=source,move=move,family=family,direction=sign,
            speed_bin_low=bucket,speed_rad_s=float(speed),
            deceleration_rad_s2=float(-sign*(v[k]-v[j])/((k-j)*dt))))
    return rows


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sources',type=Path,nargs='+',required=True)
    ap.add_argument('--v2-model',type=Path,default=Path('models/axis/sm232ae_tq10_command_v2.json'))
    ap.add_argument('--output',type=Path,required=True);args=ap.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    legs=[];windows=[];weak_traces=[];batches=0
    quantum=2*np.pi/4096
    for root in args.sources:
        if not (root/'acquisition-complete.json').exists():raise ValueError('suite incomplete: '+str(root))
        reg=json.loads((root/'registered-before-first-start.json').read_text())
        for case in reg['plans']:
            ticks,u,x,dt,p,r=load(root/case['name']);tt=(ticks-r['command_start_tick'])/1e6;v=velocity(x,dt);batches+=1
            for j,m in enumerate(case['moves']):
                start=m['start']*.02;release=(m['start']+m['width'])*.02
                stop=case['moves'][j+1]['start']*.02-.1 if j+1<len(case['moves']) else 12.9
                a=int(np.searchsorted(tt,start-.02));b=int(np.searchsorted(tt,stop));e=int(np.searchsorted(tt,release))
                seg=x[a:b];sign=1 if m['code']>0 else -1
                extreme=float(np.max(seg) if sign>0 else np.min(seg));net=float(np.rad2deg(x[b]-x[a]));extent=float(np.rad2deg(np.ptp(seg)))
                row=dict(suite=root.name,source=case['name'],move=j+1,group=m['group'],site=m.get('site',0),
                    code=m['code'],command_fraction=m['code']/32767,duration_ms=m['width']*20,
                    initial_angle_deg=float(np.rad2deg(x[a]%(2*np.pi))),
                    net_deg=net,range_deg=extent,held_deg=float(np.rad2deg(x[e]-x[a])),
                    return_from_extreme_deg=float(sign*np.rad2deg(extreme-x[b])),
                    peak_velocity_rad_s=float(np.max(abs(v[a:b]))),macro_motion_over_5deg=extent>5)
                legs.append(row)
                if m['group'] in ('weak_hold','indexed_hold'):
                    sl=(tt>=start-.05)&(tt<=min(stop,start+1.5))
                    weak_traces.append((row,tt[sl]-start,np.rad2deg(x[sl]-x[a]),u[sl]*100))
                if m['group']=='history':
                    family=f'{abs(m["code"])/32767*100:.1f}%/{m["width"]*20}ms'
                    windows+=coast_windows(ticks,x,v,dt,r['command_start_tick']+int(release*1e6),
                        r['command_start_tick']+int(stop*1e6),m['code'],family,root.name+'/'+case['name'],j+1)
    # Old matched-control zero-coast captures only; exclude +1% coast commands.
    cfg=json.loads(args.v2_model.read_text())
    for source in cfg['training_sources']:
        if '/axis-higher-coast-batched/' not in source:continue
        ticks,u,x,dt,p,r=load(source);v=velocity(x,dt);cmd=p['commands'];cp=p['manifest']['command_period_ticks']
        i=0;number=0
        while i<len(cmd):
            if cmd[i]==0:i+=1;continue
            a=i;code=cmd[i]
            while i<len(cmd) and cmd[i]==code:i+=1
            end=r['command_start_tick']+i*cp;next_index=next((k for k in range(i,len(cmd)) if cmd[k]),len(cmd))
            stop=r['command_start_tick']+next_index*cp;number+=1
            windows+=coast_windows(ticks,x,v,dt,end,min(stop,ticks[-1]),code,'old 20%/20ms',source,number)
    savecsv(args.output/'maneuvers.csv',legs);savecsv(args.output/'coast-windows.csv',windows)
    per_move=[];keys=sorted(set((r['source'],r['move'],r['family'],r['direction'],r['speed_bin_low']) for r in windows))
    for source,move,family,direction,bucket in keys:
        rr=[r for r in windows if (r['source'],r['move'],r['family'],r['direction'],r['speed_bin_low'])==(source,move,family,direction,bucket)]
        per_move.append(dict(source=source,move=move,family=family,direction=direction,speed_bin_low=bucket,
            windows=len(rr),mean_deceleration_rad_s2=float(np.mean([r['deceleration_rad_s2'] for r in rr]))))
    comparison=[]
    for family,direction,bucket in sorted(set((r['family'],r['direction'],r['speed_bin_low']) for r in per_move)):
        values=[r['mean_deceleration_rad_s2'] for r in per_move if (r['family'],r['direction'],r['speed_bin_low'])==(family,direction,bucket)]
        comparison.append(dict(family=family,direction=direction,speed_bin_low=bucket,moves=len(values),
            mean_deceleration_rad_s2=float(np.mean(values)),std_between_moves_rad_s2=float(np.std(values,ddof=1)) if len(values)>1 else 0))
    savecsv(args.output/'coast-by-move-bin.csv',per_move);savecsv(args.output/'coast-comparison.csv',comparison)
    weak=[r for r in legs if r['group'] in ('weak_hold','indexed_hold')];indexed=[r for r in weak if r['group']=='indexed_hold']
    summary=dict(captures=batches,maneuvers=len(legs),weak_holds=len(weak),indexed_holds=len(indexed),
       weak_holds_macro_over_5deg=sum(r['macro_motion_over_5deg'] for r in weak),
       indexed_initial_angle_octants=sorted(set(int(r['initial_angle_deg']//45) for r in indexed)),
       no_model_fit=True,reserved_validation_seeds_not_captured=[23240,23241],
       caveats=['No measured current or torque','Rest angle and prior motion are confounded; indexed sampling improves coverage but is not proof of cogging',
                'Coast bins use move-level means; windows within a move are not independent',
                'Old and new matched controls were recorded at different elapsed times; changes can reflect unmeasured state/drift'])
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes=plt.subplots(1,2,figsize=(13,5))
    for ax,sign in zip(axes,(1,-1)):
        for family in sorted(set(r['family'] for r in comparison)):
            rr=sorted([r for r in comparison if r['family']==family and r['direction']==sign],key=lambda r:r['speed_bin_low'])
            if rr:ax.errorbar([r['speed_bin_low']+2.5 for r in rr],[r['mean_deceleration_rad_s2'] for r in rr],
                yerr=[r['std_between_moves_rad_s2'] for r in rr],fmt='o--' if family.startswith('old') else 'o-',label=family,capsize=2)
        ax.set_title('Positive coast' if sign>0 else 'Negative coast');ax.set_xlabel('Speed (rad/s)')
        ax.set_ylabel('Deceleration (rad/s²)');ax.grid(True,alpha=.3);ax.legend(fontsize=8)
    fig.suptitle('Zero-command coast at matched speed; error bars = between-move SD, not independent windows')
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(args.output/('coast-by-history.'+ext),dpi=150)
    plt.close(fig)
    fig,axes=plt.subplots(2,3,figsize=(14,8),sharex=True)
    for ax,(sign,amp) in zip(axes.flat,[(s,a) for s in (1,-1) for a in (.01,.015,.02)]):
        for group,color,marker in (('weak_hold','tab:blue','x'),('indexed_hold','tab:orange','o')):
            rr=[r for r in weak if r['group']==group and np.sign(r['code'])==sign and abs(abs(r['command_fraction'])-amp)<.0001]
            ax.scatter([r['initial_angle_deg'] for r in rr],[r['range_deg'] for r in rr],color=color,marker=marker,label=group)
        ax.set_title(f'{sign*amp*100:+g}% hold');ax.set_yscale('symlog',linthresh=2);ax.set_xlim(0,360)
        ax.set_xlabel('Measured initial encoder phase (deg)');ax.set_ylabel('Position excursion (deg)');ax.grid(True,alpha=.3);ax.legend(fontsize=8)
    fig.suptitle('Weak-command response varies with rest state; phase dependence is not isolated from history')
    fig.tight_layout()
    for ext in ('png','svg'):fig.savefig(args.output/('weak-holds-by-angle.'+ext),dpi=150)
    plt.close(fig)
    examples=[q for q in weak_traces if q[0]['code']==-491 and q[0]['duration_ms']==200]
    if examples:
        big=max(examples,key=lambda q:q[0]['range_deg']);small=min(examples,key=lambda q:q[0]['range_deg'])
        fig,axes=plt.subplots(3,1,figsize=(11,9),sharex=True)
        for ax,q in zip(axes[:2],(big,small)):
            row,tt,xx,uu=q;ax.plot(tt,xx,color='black');ax.set_ylabel('Displacement (deg)')
            ax.set_title(f'{row["source"]}, initial phase {row["initial_angle_deg"]:.1f}°: same −1.5% / 200ms')
            ax.grid(True,alpha=.3)
        axes[2].step(big[1],big[3],where='post');axes[2].set_ylabel('Command (% FS)');axes[2].set_xlabel('Time from command onset (s)');axes[2].grid(True,alpha=.3)
        fig.suptitle('Same weak command, different rest states: large rotation versus small deflection')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/('weak-hold-contrast.'+ext),dpi=150)
        plt.close(fig)
    print(json.dumps(summary,indent=2))
    for r in comparison:
        if r['speed_bin_low']==20:print('20–25rad/s',r)


if __name__=='__main__':main()
