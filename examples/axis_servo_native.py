#!/usr/bin/env python3
"""Native checkpoint: real analog-servo Autotune.c + Control.c, frozen plant.
SIMULATION ONLY. Does not open serial, flash firmware, or claim HW agreement.
Archives exact controller sources, model, arguments, telemetry and result.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess

NAMES=('positive_gain_rad_s2_per_command','negative_gain_rad_s2_per_command',
       'positive_constant_loss_rad_s2','negative_constant_loss_rad_s2',
       'positive_viscous_s_inv','negative_viscous_s_inv','command_filter_tau_s','command_deadband_fraction')
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--controller',type=Path,default=Path('../analog-servo'))
    ap.add_argument('--model',type=Path,default=Path('models/axis/sm232ae_tq10_command_v2.json'))
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--amplitude',type=int,default=3276);ap.add_argument('--max-ticks',type=int,default=20000)
    ap.add_argument('--distance',type=int,default=512);ap.add_argument('--hold-ticks',type=int,default=1500)
    ap.add_argument('--torque-limit',type=int,default=6553);ap.add_argument('--no-plot',action='store_true')
    args=ap.parse_args();model_bytes=args.model.read_bytes();cfg=json.loads(model_bytes)
    if cfg['encoder_counts_per_rev']!=4096 or cfg['integration_step_s']!=.0005:raise ValueError('adapter requires 4096 CPR, 0.5ms model')
    if not 1<=args.amplitude<=args.torque_limit<=6553:raise ValueError('amplitude/limit must be within 20%')
    if not 1<=args.max_ticks<=20000 or not 1<=args.hold_ticks<=10000 or abs(args.distance)>65536:raise ValueError('invalid duration/distance')
    args.output.mkdir(parents=True,exist_ok=False);snap=args.output/'source';snap.mkdir()
    inputs={}
    for name in ('firmware/src/Autotune.c','firmware/src/Control.c','firmware/include/Autotune.h',
                 'firmware/include/Control.h','firmware/include/Output.h'):
        p=args.controller/name;target=snap/'analog-servo'/name;target.parent.mkdir(parents=True,exist_ok=True)
        shutil.copy2(p,target);inputs['analog-servo/'+name]=sha(target)
    for name in ('axis_servo_native.c','axis_command_step.h','axis_servo_native.py'):
        p=Path(__file__).with_name(name);shutil.copy2(p,snap/name);inputs[name]=sha(snap/name)
    (args.output/'model.json').write_bytes(model_bytes)
    exe=(args.output/'native-servo').resolve()
    # Compile the archived copies, not potentially changing worktree inputs.
    croot=snap/'analog-servo';command=['cc','-std=c99','-O2','-Wall','-Wextra','-Werror',
        '-I',str(croot/'firmware/include'),'-I',str(snap),str(snap/'axis_servo_native.c'),
        str(croot/'firmware/src/Autotune.c'),str(croot/'firmware/src/Control.c'),'-lm','-o',str(exe)]
    build=subprocess.run(command,capture_output=True,text=True)
    (args.output/'build.log').write_text(build.stdout+build.stderr);build.check_returncode()
    argv=([str(exe),str(args.amplitude),str(args.max_ticks),str(args.distance),str(args.hold_ticks),str(args.torque_limit)]+
          [repr(cfg['parameters'][n]) for n in NAMES])
    with (args.output/'telemetry.csv').open('w') as f:
        proc=subprocess.run(argv,stdout=f,stderr=subprocess.PIPE,text=True,check=True)
    (args.output/'result.json').write_text(proc.stderr)
    result=json.loads(proc.stderr)
    manifest=dict(simulated_only=True,hardware_comparison_done=False,model_sha256=sha(args.output/'model.json'),
        source_sha256=inputs,command=argv,compile_command=command,
        tick_hz=1000,plant_step_s=.0005,encoder_counts_per_rev=4096,
        feedback='nearest integer encoder count then count delta per tick; never simulator velocity truth',
        state='continuous plant/filter state through relay, 500ms coast, two closed-loop position steps',
        gain_units='raw Q8.8, velocity integral per control tick; effort signed code/32767, not current',
        result=result)
    (args.output/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    if not args.no_plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        with (args.output/'telemetry.csv').open() as f:rows=list(csv.DictReader(f))
        tt=[int(r['tick'])/1000 for r in rows]
        fig,axes=plt.subplots(3,1,figsize=(12,9),sharex=True)
        axes[0].plot(tt,[int(r['reference_count']) for r in rows],label='Reference',color='gray')
        axes[0].plot(tt,[int(r['position_count']) for r in rows],label='Simulated encoder',color='tab:blue')
        axes[0].set_ylabel('Position (counts)');axes[0].legend()
        axes[1].plot(tt,[int(r['velocity_count_per_tick']) for r in rows]);axes[1].set_ylabel('Encoder delta / tick')
        axes[2].plot(tt,[int(r['effort_code'])/32767*100 for r in rows]);axes[2].set_ylabel('Effort (% FS)');axes[2].set_xlabel('Time (s)')
        for ax in axes:ax.grid(True,alpha=.3)
        fig.suptitle('SIMULATION ONLY — unmodified analog-servo autotune + Control.c, frozen command-v2 plant')
        fig.tight_layout()
        for ext in ('png','svg'):fig.savefig(args.output/('native-tune.'+ext),dpi=150)
        plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
