#!/usr/bin/env python3
"""Develop v3 offline: same-effort continuous replays, never powered control.

The extra state stores elastic friction load, with velocity-dependent plastic
bounds. It is a hypothesis, not an identified mechanical component. Complete
suites carry state through recorded zero-output gaps by default. Where preceding
input history is unavailable, explicitly assume rest/zero load. Initial states
and position offsets are never fitted. Legacy ramp data are an unqualified
cross-firmware diagnostic, never fitting data.
"""
import argparse
import csv
import json
from pathlib import Path
import shutil
import numpy as np
from scipy.optimize import least_squares
from axis_fit_controller_workload import Batch, NAMES, arrays, load, select_blocks, sha, save
from axis_model_fit import load as load_effort
from axis_residual_diagnostics import verify_effort_capture

ROOT = Path(__file__).resolve().parents[1]
EXTRA = ['positive_elastic_yield_rad_s2', 'negative_elastic_yield_rad_s2',
         'elastic_stiffness_s_inv2', 'yield_velocity_scale_rad_s']
Q = 2*np.pi/4096


def velocity(x, dt=.0005):
    i = np.arange(len(x)); a = np.maximum(i-10, 0); b = np.minimum(i+10, len(x)-1)
    return (x[b]-x[a])/((b-a)*dt)


def legacy(path, sign):
    data = json.loads(path.read_text())
    meta_path = path.with_name(path.name+'.meta.json')
    meta = json.loads(meta_path.read_text())
    if not meta['completed'] or sign not in (-1, 1):
        raise ValueError('requires completed legacy capture and explicit sign')
    seq = np.array([f['seq'] for f in data], dtype=np.int64)
    if np.any(np.diff(seq) != 1) or any(f['fault_latched'] for f in data):
        raise ValueError('legacy gaps/faults: do not interpolate missing effort')
    if any(f.get('excitation') for f in data):
        raise ValueError('legacy excitation semantics need a separate decoder')
    expected = f'output.polarity={sign}\r\n'
    if expected not in meta['entry_settings'] or expected not in meta['exit_settings']:
        raise ValueError('explicit sign differs from recorded axis polarity')
    count = np.array([f['position'] for f in data], dtype=float)
    if np.ptp(count[:200]) > 2:
        raise ValueError('nonquiet legacy capture origin')
    effort = np.repeat([sign*f['torque']/32767 for f in data], 2)[:-1]
    x = np.interp(np.arange(len(effort))*.0005, (seq-seq[0])*.001, (count-count[0])*Q)
    reference = np.repeat([f['cmd_position']-count[0] for f in data], 2)[:-1]*Q
    moving = np.flatnonzero([f['cmd_velocity'] != 0 for f in data])
    steps = np.flatnonzero(np.abs(np.diff([f['cmd_position'] for f in data])) >= 500)+1
    if not len(moving) or not len(steps):
        raise ValueError('requires recorded step and velocity ramp')
    windows = [('step-settling', max(0, steps[0]*.001-.1), steps[0]*.001+1.0),
               ('ramp-settling', max(0, moving[0]*.001-.1), min(len(x)*.0005, moving[-1]*.001+1.0))]
    inventory = {str(p): sha(p) for p in [path, meta_path]}
    return (effort, x, inventory), reference, windows


def plot_case(out, label, episode, reference, windows, baseline, candidate, p, extras, qualification, candidate_p=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    u, x, _ = episode
    uu = np.ascontiguousarray(u[None, :]); ll = np.array([len(u)], dtype=np.int32)
    predictions = {'Real': x, 'v2': baseline.simulate(uu, ll, p)[0],
                   'v3 experimental': candidate.simulate(uu, ll, np.r_[p, extras] if candidate_p is None else candidate_p)[0]}
    tt = np.arange(len(x))*.0005
    colors = {'Real': 'black', 'v2': 'tab:orange', 'v3 experimental': 'tab:blue'}
    metrics = {}
    fig, axes = plt.subplots(4, len(windows), figsize=(15, 10), squeeze=False, layout='constrained')
    for col, (name, start, stop) in enumerate(windows):
        keep = (tt >= start) & (tt <= stop)
        for model, predicted in predictions.items():
            color = colors[model]
            axes[0, col].plot(tt[keep]-start, predicted[keep]/Q, label=model, color=color)
            axes[1, col].plot(tt[keep]-start, velocity(predicted)[keep], color=color)
            if model != 'Real':
                axes[2, col].plot(tt[keep]-start, (predicted[keep]-x[keep])/Q, color=color, label=model)
                metrics[name+'/'+model] = dict(position_rmse_counts=float(np.sqrt(np.mean(((predicted[keep]-x[keep])/Q)**2))),
                    velocity_rmse_rad_s=float(np.sqrt(np.mean((velocity(predicted)[keep]-velocity(x)[keep])**2))))
        axes[0, col].plot(tt[keep]-start, reference[keep]/Q, '--', color='0.5', label='Recorded reference')
        axes[3, col].step(tt[keep]-start, u[keep]*32767, where='post', color='tab:green')
        axes[0, col].set_title(name+' — continuous replay, no local re-zero')
        for row, ylabel in enumerate(['Position (counts from capture origin)', 'Velocity (rad/s, same 10 ms estimator)',
                                      'Prediction minus real (counts)', 'Recorded effort (axis-aligned code)']):
            axes[row, col].set_ylabel(ylabel); axes[row, col].grid(alpha=.2)
        axes[0, col].legend(fontsize=8); axes[2, col].legend(fontsize=8)
        axes[3, col].set_xlabel('Time from displayed window start (s); no fitted shift')
    fig.suptitle(label+' · real/v2/v3 receive the same recorded effort\n'+qualification, fontsize=11)
    fig.savefig(out/(label+'.png'), dpi=150); plt.close(fig)
    np.savez_compressed(out/(label+'.npz'), t=tt, real=x, v2=predictions['v2'], v3=predictions['v3 experimental'], effort=u, reference=reference)
    return metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sources', type=Path, nargs='+', required=True)
    ap.add_argument('--controller', type=Path, nargs='+', required=True)
    ap.add_argument('--legacy-frames', type=Path)
    ap.add_argument('--legacy-effort-sign', type=int, choices=[-1, 1])
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--family', choices=['speed-yield', 'slip-yield', 'age-yield'], default='speed-yield')
    ap.add_argument('--promote-seen-coasts', type=Path, nargs='*', default=[],
                    help='Explicitly promote previously seen mixed-pulse checks to training')
    ap.add_argument('--normalization', choices=['amplitude', 'tolerance'], default='amplitude')
    ap.add_argument('--carry-suite-state', action='store_true', help='Carry physical state across complete targeted suites and controller training prefixes (default)')
    ap.add_argument('--independent-capture-rest-assumption',dest='carry_suite_state',action='store_false',
                    help='Explicit diagnostic override: assume rest/zero elastic load independently per capture')
    ap.set_defaults(carry_suite_state=True)
    ap.add_argument('--warm-start', type=Path, help='Frozen candidate from the same family')
    ap.add_argument('--refine-gains', action='store_true', help='Allow only +/- command gains to move within eight percent of v2')
    ap.add_argument('--max-nfev', type=int, default=45)
    ap.add_argument('--starts', type=int, default=6)
    args = ap.parse_args(); out = args.output; out.mkdir(parents=True, exist_ok=False)
    if args.legacy_frames and args.legacy_effort_sign is None:
        raise ValueError('legacy effort convention must be specified, never fitted')
    seed_path = ROOT/'models/axis/sm232ae_tq10_command_v2.json'; seed = json.loads(seed_path.read_text())
    p = np.array([seed['parameters'][n] for n in NAMES]); sources = out/'source'; sources.mkdir()
    extra_names = (EXTRA[:3]+['moving_loss_elastic_fraction'] if args.family == 'slip-yield' else
                   EXTRA+(['contact_healing_time_s', 'contact_renewal_distance_rad'] if args.family == 'age-yield' else []))
    kernel_name = {'speed-yield': 'axis_command_model_v3.c', 'slip-yield': 'axis_command_model_v3_slip.c',
                   'age-yield': 'axis_command_model_v3_age.c'}[args.family]
    for name in ['axis_model_v3.py', 'axis_command_model_v3.c', 'axis_command_model_v3_slip.c', 'axis_command_model_v3_age.c', 'axis_command_model_v2.c',
                 'axis_fit_controller_workload.py', 'axis_model_fit.py', 'axis_v3_chain.py', 'axis_v3_compare.py',
                 'axis_v3_stateful.c', 'axis_command_step.h', 'axis_residual_diagnostics.py', 'axis_model_fit_v2.py']:
        shutil.copy2(ROOT/'examples'/name, sources/name)
    shutil.copy2(seed_path, out/'frozen-v2.json')
    if sha(sources/'axis_command_model_v2.c') != seed['kernel_sha256']:
        raise ValueError('frozen v2 kernel changed')
    (out/'v2').mkdir(); (out/'v3').mkdir()
    baseline = Batch(sources/'axis_command_model_v2.c', out/'v2')
    candidate = Batch(sources/kernel_name, out/'v3')
    old = []; target = []; train = []; dev = []; inventory = {}; splits = {}
    target_gaps = []; training_gaps = []
    def carry_coordinates(cases, episodes):
        from axis_v3_chain import phase_gap
        gaps=[]; converted=[]; origin=None; end=None;last_count=None
        for case,episode in zip(cases,episodes):
            count=np.fromfile(case/'stream-0.i64le',dtype='<i8')
            run=json.loads((case/'get_run.json').read_text())
            descriptor=next(d for d in run['streams'] if d['stream_id']==0)
            tick=descriptor['first_sample_tick'];u=episode[0]
            if descriptor['period_ticks']!=500 or u[0]!=0 or u[-1]!=0:
                raise ValueError('state carry requires aligned 2kHz data and zero-output pre/postroll')
            if origin is None:origin=int(count[0])
            gap=phase_gap(end,last_count,tick,int(count[0]))
            converted.append((u,(count-origin)*Q,episode[2]));gaps.append(gap)
            end=tick+(len(count)-1)*500;last_count=int(count[-1])
        return converted,gaps
    def effort_episode(case):
        _, u, x, dt, _, _ = load_effort(case)
        if dt != .0005 or np.ptp(x[:160]) > 4*Q: raise ValueError('requires quiet 2kHz origin')
        inventory[str(case)] = verify_effort_capture(case)
        return u, x-x[0], inventory[str(case)]
    old_paths = list(map(Path, seed['training_sources']))+args.promote_seen_coasts
    if len({s.resolve() for s in old_paths}) != len(old_paths):raise ValueError('duplicate old training source')
    for source in old_paths: old.append(effort_episode(source))
    for root in args.sources:
        reg = json.loads((root/'registered-before-first-start.json').read_text())
        cases=[root/item['name'] for item in reg['plans']]
        episodes=[effort_episode(case) for case in cases]
        if args.carry_suite_state:episodes,gaps=carry_coordinates(cases,episodes);target_gaps+=gaps
        target+=episodes
    for root in args.controller:
        a, b, excluded = select_blocks(root); splits[str(root)] = dict(training=list(map(str,a)), development=list(map(str,b)), excluded=excluded)
        for cases, dataset in [(a, train), (b, dev)]:
            episodes=[load(case) for case in cases]
            for case,episode in zip(cases,episodes):inventory[str(case)]=episode[2]
            if args.carry_suite_state and dataset is train:
                ids=[int(case.name.split('-')[-1]) for case in cases]
                if ids!=list(range(ids[0],ids[-1]+1)):raise ValueError('cannot bridge excluded controller captures')
                episodes,gaps=carry_coordinates(cases,episodes);training_gaps+=gaps
            dataset+=episodes
    legacy_data = legacy(args.legacy_frames, args.legacy_effort_sign) if args.legacy_frames else None
    last_dev = Path(list(splits.values())[-1]['development'][-1])
    control_episode = load(last_dev)
    telemetry = np.fromfile(last_dev/'stream-2.i32le', dtype='<i4').reshape(-1, 2)
    reference = np.repeat(telemetry[:,1], 2)[:len(control_episode[0])]*Q
    pre = json.loads((last_dev/'plan.json').read_text())['manifest']['capture_pre_roll_ticks']/1e6
    def plots(extras, folder, gains=None):
        if args.carry_suite_state:return  # Use the cohort comparer, not reset-state closeups.
        folder.mkdir()
        result = {'controller': plot_case(folder, 'controller-settling', control_episode, reference,
            [('first step', pre, pre+.6), ('last return', max(pre, len(reference)*.0005-1.6), len(reference)*.0005)],
            baseline, candidate, p, extras, 'Current rig; seen development recording; no controller re-tuning; '+args.family,
            candidate_p=np.r_[p[:2] if gains is None else gains, p[2:], extras])}
        if legacy_data:
            e, ref, windows = legacy_data
            result['legacy'] = plot_case(folder, 'legacy-ramp-and-settling', e, ref, windows,
                baseline, candidate, p, extras,
                f'Legacy firmware; effort sign {args.legacy_effort_sign:+d} from recorded polarity; scale transfer unqualified; NOT fitting data; {args.family}',
                candidate_p=np.r_[p[:2] if gains is None else gains, p[2:], extras])
        save(folder/'metrics.json', result)
    initial = np.array([350., 350., 10000., 3.])
    lower = np.array([0., 0., 500., .2]); upper = np.array([900., 900., 100000., 12.])
    if args.family == 'slip-yield':
        initial[-1] = .5; lower[-1] = .05; upper[-1] = .95
    rng = np.random.default_rng(23260)
    starts = [initial]+[np.array([rng.uniform(100,700), rng.uniform(100,700),
        np.exp(rng.uniform(np.log(1500),np.log(50000))), np.exp(rng.uniform(np.log(.5),np.log(10)))]) for _ in range(args.starts-1)]
    if args.family == 'slip-yield':
        for start in starts[1:]: start[-1] = rng.uniform(.1, .9)
    if args.family == 'age-yield':
        initial = np.r_[initial, .5, .05]
        starts = [np.r_[s, .5, .05] for s in starts]
        for start in starts[1:]:
            start[-2:] = [np.exp(rng.uniform(np.log(.05), np.log(5))), np.exp(rng.uniform(np.log(.003), np.log(.3)))]
        lower = np.r_[lower, .02, .0005]; upper = np.r_[upper, 20., 1.]
    warm = None
    if args.warm_start:
        warm = json.loads(args.warm_start.read_text())
        if warm.get('family', 'speed-yield') != args.family or warm['kernel_sha256'] != sha(sources/kernel_name):
            raise ValueError('warm-start family/kernel mismatch')
        if warm['original_v2_sha256'] != sha(seed_path):
            raise ValueError('warm-start seed mismatch')
        for n in (NAMES[2:] if args.refine_gains else NAMES):
            if warm['parameters'][n] != seed['parameters'][n]:
                raise ValueError('warm-start changes a fixed parameter: '+n)
        initial = np.array([warm['parameters'][n] for n in extra_names])
        starts = [initial]+[np.clip(initial*rng.uniform(.9,1.1,len(initial)), lower+1e-7, upper-1e-7) for _ in range(args.starts-1)]
        shutil.copy2(args.warm_start, out/'warm-start.json')
    gains = np.array([warm['parameters'][n] for n in NAMES[:2]]) if warm else p[:2]
    if args.refine_gains:
        starts = [np.r_[gains,s] for s in starts]
        lower = np.r_[p[:2]*.92,lower]; upper = np.r_[p[:2]*1.08,upper]
    def decode(free):
        return (np.r_[free[:2],p[2:]],free[2:]) if args.refine_gains else (p,free)
    save(out/'registered.json', dict(status='seen offline development, not physical validation', family=args.family,
        refine_gains=args.refine_gains, carry_suite_state=args.carry_suite_state,
        state_carry_gaps=dict(targeted=target_gaps,controller_training=training_gaps),
        old_initial_state='Independent rest/zero-load assumptions where preceding input history is unavailable.',
        warm_start_sha256=sha(args.warm_start) if args.warm_start else None,
        source_sha256={s.name:sha(s) for s in sorted(sources.iterdir())},
        seed_sha256=sha(seed_path), kernel_sha256=sha(sources/kernel_name),
        old_sources=list(map(str,old_paths)), promoted_seen_coasts=list(map(str,args.promote_seen_coasts)),
        normalization=args.normalization,
        tolerance_scales=dict(position_floor_deg=5,position_span_fraction=.05,velocity_floor_rad_s=1,velocity_peak_fraction=.05)
            if args.normalization=='tolerance' else None,
        targeted_sources=list(map(str,args.sources)), controller_splits=splits,
        raw_inputs=inventory, legacy_diagnostic_inventory=legacy_data[0][2] if legacy_data else {},
        fixed_parameters={n:seed['parameters'][n] for n in NAMES[2:]} if args.refine_gains else seed['parameters'],
        extra_names=extra_names, bounds=[lower.tolist(), upper.tolist()],
        starts=[s.tolist() for s in starts], objective='equal family / equal capture normalized position and 10ms velocity; extra parameters and optionally bounded gains fitted',
        initial_state='one rest/zero-elastic-load assumption per capture, never per maneuver; no fitted nuisance states',
        independent_validation=False, no_hardware_interface=True))
    plots(initial, out/'initial-plots', gains=gains if args.refine_gains else None)
    print('State-carry fit: use axis_v3_chain.py for plots' if args.carry_suite_state else 'Initial plots: '+str(out/'initial-plots'), flush=True)
    stateful=None
    if args.carry_suite_state:
        from axis_v3_chain import Stateful, initial_state
        stateful=Stateful(sources/'axis_v3_stateful.c',out/'stateful')
    families = []
    for data,gaps in [(old,None),(target,target_gaps),(train,training_gaps)]:
        u, x, ll, mask, _ = arrays(data)
        spans = np.array([np.ptp(e[1]) for e in data])
        scales = (np.maximum(.05*spans,np.deg2rad(5)) if args.normalization=='tolerance' else np.maximum(spans,.05))[:,None]
        w = 1/np.sqrt(mask.sum(axis=1)*len(ll))[:,None]/scales
        vv = (x[:,20:]-x[:,:-20])/.01; vm = mask[:,10:-10].copy()
        for i, length in enumerate(ll): vm[i, max(0,length-20):] = False
        peaks = np.array([np.percentile(abs(vv[i,:n-20]),95) for i,n in enumerate(ll)])
        vs = (np.maximum(.05*peaks,1) if args.normalization=='tolerance' else np.maximum(peaks,.5))[:,None]
        vw = 1/np.sqrt(vm.sum(axis=1)*len(ll))[:,None]/vs
        families.append((u,x,ll,mask,w,vv,vm,vw,gaps))
    def residual(free):
        base,extras = decode(free)
        parts = []
        for u,x,ll,mask,w,vv,vm,vw,gaps in families:
            params=np.r_[base,extras]
            if stateful is None or gaps is None:
                pred = candidate.simulate(u,ll,params)
            else:
                pred=np.zeros_like(x);state=initial_state()
                family_code={'speed-yield':1,'slip-yield':2,'age-yield':3}[args.family]
                for i,(length,gap) in enumerate(zip(ll,gaps)):
                    if gap is None:state=initial_state()
                    else:stateful.gap(family_code,gap,params,state)
                    pred[i,:length]=stateful.run(family_code,u[i,:length],params,state)[0]
            parts.append(((pred-x)*w)[mask]); pv=(pred[:,20:]-pred[:,:-20])/.01
            parts.append(.5*((pv-vv)*vw)[vm])
        return np.concatenate(parts)
    results = []; best = None
    for i, start in enumerate(starts):
        fit = least_squares(residual, start, bounds=(lower,upper), diff_step=.001, x_scale='jac',
            max_nfev=args.max_nfev, ftol=1e-5, xtol=1e-5, gtol=1e-5)
        base,extras = decode(fit.x)
        row = dict(start=i, extras=extras.tolist(), command_gains=base[:2].tolist(), cost=float(fit.cost), nfev=fit.nfev, success=bool(fit.success))
        results.append(row); save(out/'starts.json', results); print(json.dumps(row), flush=True)
        if best is None or row['cost'] < best['cost']:
            best = row
            config = dict(schema='axis-command-model-v3', family=args.family, status='experimental, not promoted or physically validated',
                parameters=dict(zip(list(NAMES)+extra_names, map(float,np.r_[base,extras]))),
                moving_gains_refined=args.refine_gains, state_carry_fitted=args.carry_suite_state,
                kernel_sha256=sha(sources/kernel_name), original_v2_sha256=sha(seed_path),
                integration_step_s=.0005, encoder_counts_per_rev=4096, selection='minimum joint training objective only',
                initial_state=('Rest, zero load, fully healed at each suite origin; state carried thereafter. Older standalone inputs retain independent rest assumptions.'
                    if args.carry_suite_state else 'Diagnostic independent rest/zero-load/fully-healed assumption per capture.'),
                initial_state_limitation='Actual preload/contact strength at the first known input are unmeasured.')
            save(out/'candidate-model.json', config)
            plots(extras, out/f'best-after-{i:02d}', gains=base[:2])
    save(out/'summary.json', dict(chosen=best, old_cases=len(old), targeted_cases=len(target),
        controller_training_cases=len(train), controller_development_cases=len(dev), independent_validation=False))


if __name__ == '__main__': main()
