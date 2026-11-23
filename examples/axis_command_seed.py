#!/usr/bin/env python3
"""Offline rough direction-specific command-normalized seed; not physical ID.

Uses only the large-probe bundle (not reserved validation data). Fits coast-down
integral balances with nonnegative constant/viscous deceleration. Estimates gain
from velocity plus modeled drag impulse through 10ms after command pulse end.
Uploaded command is treated as a rectangle; unmeasured RC/drive dynamics and
cogging are not identified. Save as a provisional empirical seed, not motor SI
inertia/friction. NumPy and SciPy required. Never connects to hardware.
"""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from scipy.optimize import lsq_linear


def estimate(root):
    trials = []
    balances = {1: [], -1: []}
    for case in sorted(Path(root).glob('batch-*')):
        report = json.loads((case/'report.json').read_text())
        if not report['passed'] or report['state'] != 'COMPLETE':
            raise ValueError('invalid capture')
        run = json.loads((case/'get_run.json').read_text())
        desc = run['streams'][0]
        counts = np.fromfile(case/'stream-0.i64le', dtype='<i8')
        if len(counts) != desc['sample_count']:
            raise ValueError('sample count mismatch')
        dt = desc['period_ticks']/1e6
        theta = counts*2*np.pi/4096
        t = (desc['first_sample_tick']+np.arange(len(counts))*desc['period_ticks'])/1e6
        k = max(1, round(.0025/dt))
        idx = np.arange(len(theta)); lo = np.maximum(0,idx-k); hi = np.minimum(len(theta)-1,idx+k)
        velocity = (theta[hi]-theta[lo])/((hi-lo)*dt)
        for m in csv.DictReader((case/'pulse-metrics.csv').open()):
            sign = int(m['direction']); start = int(m['command_start_tick'])/1e6
            end = int(m['command_end_tick'])/1e6
            speed = velocity*sign
            peak = np.max(speed[(t>=start)&(t<=end+.05)])
            width = round(.02/dt)
            for a in range(np.searchsorted(t,end+.01),np.searchsorted(t,end+.4),max(1,round(.01/dt))):
                b=a+width
                if b>=len(t) or speed[a]<.2*peak or speed[b]<.2*peak:
                    continue
                dx=(theta[b]-theta[a])*sign
                balances[sign].append((t[b]-t[a], dx, speed[a]-speed[b]))
            a = int(np.searchsorted(t,start)); b = int(np.searchsorted(t,end+.01))
            moving_time=np.sum(speed[a:b]>.05*peak)*dt
            trials.append(dict(sign=sign, dv=speed[b]-speed[a],
                               dx=(theta[b]-theta[a])*sign, moving_time=moving_time,
                               command_impulse=abs(int(m['command_code']))/32767*(end-start)))
    result={}
    for sign,name in ((1,'positive'),(-1,'negative')):
        data=np.array(balances[sign]); x=data[:,:2]; y=data[:,2]
        scales=np.linalg.norm(x,axis=0)
        fit=lsq_linear(x/scales,y,bounds=(0,np.inf))
        if not fit.success:
            raise ValueError(fit.message)
        c,b=fit.x/scales
        gains=[(r['dv']+c*r['moving_time']+b*r['dx'])/r['command_impulse']
               for r in trials if r['sign']==sign]
        result[name]=dict(coast_coulomb_deceleration_rad_s2=float(c),
                          coast_viscous_rate_s_inv=float(b),
                          apparent_command_acceleration_gain_rad_s2_per_full_scale=float(np.median(gains)),
                          gain_interquartile_range=[float(v) for v in np.percentile(gains,[25,75])],
                          trials=len(gains), coast_balance_windows=len(data),
                          coast_balance_rmse_rad_s=float(np.sqrt(np.mean((x@np.array([c,b])-y)**2))))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bundle',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    directional=estimate(args.bundle)
    rotor_inertia=9.3e-5  # User-supplied motor datasheet; total shaft inertia unmeasured.
    rotor_equivalents={name: dict(
        torque_gain_Nm_per_fullscale=rotor_inertia*m['apparent_command_acceleration_gain_rad_s2_per_full_scale'],
        effective_constant_drag_Nm=rotor_inertia*m['coast_coulomb_deceleration_rad_s2'],
        effective_viscous_drag_Nm_s_per_rad=rotor_inertia*m['coast_viscous_rate_s_inv'])
        for name,m in directional.items()}
    seed=dict(status='provisional, unvalidated command-normalized seed',motor='SM232AE',drive='TQ10',
              encoder_counts_per_rev=4096,decode_mult=4,command_full_scale_code=32767,
              physical_parameters=dict(rotor_inertia_kg_m2=rotor_inertia,
                                       total_shaft_inertia_kg_m2=None,
                                       back_emf_constant_reported_V_per_rad_s=0.310,
                                       back_emf_voltage_convention='line-to-line, peak of sine wave; user-confirmed datasheet notes',
                                       resistance_line_to_line_ohm=7.50,
                                       inductance_line_to_line_H=2.90e-3,
                                       inductance_tolerance_fraction=0.30,
                                       inductance_measurement_frequency_Hz=1000,
                                       continuous_current_trapezoidal_A=2.0,
                                       trapezoidal_current_convention='DC current through an active pair of phases',
                                       pole_count=4, pole_pairs=2,
                                       peak_current_trapezoidal_A=7.2,
                                       peak_current_note='motor datasheet rating, not measured current or configured drive limit',
                                       provenance='user-provided SM232AE datasheet values',
                                       torque_constant_Nm_A=0.307,
                                       torque_constant_current_convention='DC current through active pair, trapezoidal commutation',
                                       drive_command_current_gain_A_per_V=1.0,
                                       dc_bus_voltage_V=170.0,
                                       dc_bus_voltage_note='approximate installed bus voltage reported by user',
                                       drive_gain_provenance='user quote from TQ10X manual: 1.0 A per command volt, +/-10 V input',
                                       actual_current_per_full_scale_A=None),
              electrical_equivalent=dict(
                  assumption='balanced wye-equivalent, independent windings; actual winding topology/mutual inductance unconfirmed',
                  resistance_per_phase_ohm=3.75,
                  inductance_per_phase_H=1.45e-3,
                  inductance_tolerance_fraction=0.30,
                  winding_L_over_R_seconds=2.90e-3/7.50,
                  back_emf_per_phase_peak_V_per_rad_s=0.310/(3**0.5),
                  back_emf_selection_status='line-to-line sine peak confirmed; convert by sqrt(3)',
                  note='phase Ke is not numerically the aggregate trapezoidal pair-current Kt'),
              nominal_physical_predictions=dict(
                  assumption='3.3 V full-scale differential output, 1 A/V drive gain; datasheet rotor inertia only',
                  torque_per_full_scale_Nm=0.307*3.3,
                  acceleration_gain_rad_s2_per_fullscale=0.307*3.3/rotor_inertia),
              rotor_only_equivalent_parameters=dict(
                  assumption='total shaft inertia equals datasheet rotor inertia; not measured torque',
                  directional=rotor_equivalents),
              hypothesis=dict(current_per_full_scale_A=3.3,
                              command_voltage_per_full_scale_V=3.3,
                              differential_stage_voltage_gain=1.0,
                              status='nominal using manual 1 A/V; assumes unity differential-stage gain and 3.3 V swing; voltage/current path unmeasured'),
              model='dv/dt = G_sign(u)*u - C_sign(v)*sign(v) - B_sign(v)*v; stick at rest below friction',
              warning='Coast fit only; gain impulse approximation ignores unmeasured drive/filter dynamics. '
                      'No full trajectory fit or independent validation. Rotor-only equivalent torque/drag '
                      'does not establish actual drive torque, intrinsic friction, or calibrated current gain.',
              source=str(args.bundle),directional=directional)
    args.output.write_text(json.dumps(seed,indent=2)+'\n')
    print(json.dumps(seed,indent=2))


if __name__=='__main__':
    main()
