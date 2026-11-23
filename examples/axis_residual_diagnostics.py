#!/usr/bin/env python3
"""Read-only residual stratification; no fitting, serial access or motion backend.

Compare registered predictions with continuous frozen-v2 replays. Position-error
changes across maneuvers are descriptive differences, not per-maneuver resets.
Encoder-stationary windows do not prove absence of sub-count shaft motion.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import numpy as np
from axis_model_fit import load as load_effort
from axis_model_fit_v2 import Model, NAMES
from axis_fit_controller_workload import load as load_servo, select_blocks

ROOT = Path(__file__).resolve().parents[1]
CPR = 4096
QUANTUM = 2 * np.pi / CPR


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_json(path):
    return json.loads(Path(path).read_text())


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def write_csv(path, rows):
    if not rows:
        return
    with Path(path).open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def verify_phase_chain(captures):
    """Require consistent clocks and encoder endpoints, not boot_id alone.

    The firmware's boot ID can repeat after a reset. These captures have quiet
    gaps, so a shared phase frame also requires adjacent encoder endpoints to
    agree within one count. This is not a claim about hidden friction states.
    """
    ordered = sorted(captures, key=lambda c: c['first_tick'])
    for previous, current in zip(ordered, ordered[1:]):
        if previous['boot_id'] != current['boot_id'] or current['first_tick'] <= previous['last_tick']:
            raise ValueError('phase frame has inconsistent clocks/boot identity')
        if abs(current['first_count'] - previous['last_count']) > 1:
            raise ValueError('phase frame has unverified encoder continuity')


def static_bounds(parameters):
    """Steady command-code bounds for frozen v2 at zero velocity."""
    p = parameters
    bounds = []
    for direction in ('negative', 'positive'):
        bounds.append(32767 * (p['command_deadband_fraction'] +
            p[direction + '_constant_loss_rad_s2'] / p[direction + '_gain_rad_s2_per_command']))
    return -bounds[0], bounds[1]


def verify_effort_capture(case):
    """Validate raw input identity before the legacy loader constructs arrays."""
    case = Path(case)
    report = read_json(case / 'report.json')
    run = read_json(case / 'get_run.json')
    plan = read_json(case / 'plan.json')
    manifest = plan['manifest']
    if not report['passed'] or report['state'] != 'COMPLETE' or report.get('errors'):
        raise ValueError('failed integrity report: ' + str(case))
    if not run['capture_complete'] or run.get('fault_reason') is not None:
        raise ValueError('incomplete/faulted run')
    if manifest['output_mode'] != 'analog_effort' or manifest['analog_enabled']:
        raise ValueError('requires uploaded effort, not references or ADC')
    if manifest['analog_output_full_scale'] != 32767:
        raise ValueError('unsupported effort scale')
    descriptor = next(s for s in run['streams'] if s['stream_id'] == 0)
    reported = report['streams']['0']
    raw = case / 'stream-0.i64le'
    if sha(raw) != descriptor['sha256'] or sha(raw) != reported['sha256']:
        raise ValueError('encoder hash mismatch')
    if raw.stat().st_size != descriptor['sample_count'] * 8 or descriptor['sample_count'] != reported['sample_count']:
        raise ValueError('encoder length mismatch')
    if descriptor['period_ticks'] != manifest['encoder_sample_period_ticks']:
        raise ValueError('encoder period mismatch')
    if descriptor['first_sample_tick'] != run['capture_start_tick'] or descriptor['first_sample_tick'] != reported['first_tick']:
        raise ValueError('capture origin mismatch')
    # The wire contract hashes commands widened to i64, not their upload width.
    digest = hashlib.sha256(np.asarray(plan['commands'], dtype='<i8').tobytes()).hexdigest()
    if digest != run['command_hash'] or len(plan['commands']) != manifest['command_sample_count']:
        raise ValueError('command hash/count mismatch')
    if plan['run_id'] != run['run_id']:
        raise ValueError('run identity mismatch')
    return {name: dict(sha256=sha(case / name), bytes=(case / name).stat().st_size)
            for name in ('stream-0.i64le', 'plan.json', 'get_run.json', 'report.json')}


def position_velocity(position, dt):
    """10ms central-difference velocity, for both measured and predicted data."""
    i = np.arange(len(position))
    lo = np.maximum(i - 10, 0)
    hi = np.minimum(i + 10, len(position) - 1)
    return (position[hi] - position[lo]) / ((hi - lo) * dt)


def quadratic_kinematics(position, dt):
    """Local encoder smoothing, not a plant fit. Return midpoint v, a and error."""
    position = np.asarray(position)
    if len(position) < 3:
        raise ValueError('quadratic derivative needs at least three samples')
    scale = (len(position) - 1) * dt / 2
    xi = np.linspace(-1, 1, len(position))
    design = np.column_stack([np.ones(len(xi)), xi, xi * xi])
    centered = position - position[len(position) // 2]
    c = np.linalg.lstsq(design, centered, rcond=None)[0]
    return float(c[1] / scale), float(2 * c[2] / scale**2), float(np.sqrt(np.mean((design @ c - centered)**2)) / QUANTUM)


def driven_windows(t, position, moves, parameters, metadata):
    """20ms kinematic windows inside pulses, 5ms clear of either edge.

    Force ratios assume frozen-v2 drag; they do NOT separately identify input
    nonlinearity and missing friction. Keep per-move grouping for inference.
    """
    dt = float(t[1] - t[0])
    rows = []
    for number, move in enumerate(moves, 1):
        start = move['start'] * .02
        release = (move['start'] + move['width']) * .02
        for origin in np.arange(start + .005, release - .025 + 1e-10, .02):
            a = int(np.searchsorted(t, origin - 1e-10))
            segment = position[a:a+41]
            if len(segment) != 41 or np.ptp(segment) < 8 * QUANTUM:
                continue
            v, acceleration, fit_error = quadratic_kinematics(segment, dt)
            if abs(v) < 1 or v * move['code'] <= 0:
                continue
            direction = 'positive' if v > 0 else 'negative'
            input_direction = 'positive' if move['code'] > 0 else 'negative'
            force = parameters[input_direction + '_gain_rad_s2_per_command'] * np.sign(move['code']) * max(abs(move['code'])/32767 - parameters['command_deadband_fraction'], 0)
            drag = np.sign(v) * parameters[direction + '_constant_loss_rad_s2'] + parameters[direction + '_viscous_s_inv'] * v
            rows.append(dict(**metadata, move=number, group=move['group'], code=move['code'],
                start_s=float(t[a]), speed_rad_s=v, speed_bin_low=int(abs(v)//5)*5,
                acceleration_rad_s2=acceleration, frozen_acceleration_rad_s2=float(force-drag),
                acceleration_residual_rad_s2=float(acceleration-force+drag),
                conditional_force_ratio=float((acceleration+drag)/force),
                kinematic_fit_rmse_counts=fit_error))
    return rows


def maneuver_rows(t, measured, predicted, predicted_velocity, moves, metadata):
    rows = []
    for j, move in enumerate(moves):
        start = move['start'] * .02
        release = (move['start'] + move['width']) * .02
        stop = moves[j + 1]['start'] * .02 - .1 if j + 1 < len(moves) else min(12.9, t[-1])
        a = int(np.searchsorted(t, start - .02))
        b = int(np.searchsorted(t, stop)) - 1
        e = int(np.searchsorted(t, release))
        quiet = measured[(t >= start - .2) & (t <= start - .02)]
        direction = 1 if move['code'] > 0 else -1
        prior = int(np.sign(moves[j - 1]['code'])) if j else 0
        error = predicted - (measured - measured[0])
        segment = measured[a:b + 1]
        extreme = np.max(segment) if direction > 0 else np.min(segment)
        rows.append(dict(**metadata, move=j + 1, group=move['group'], site=move.get('site', 0),
            code=move['code'], direction=direction, duration_ms=move['width'] * 20,
            previous_direction=prior, previous_code=moves[j - 1]['code'] if j else 0,
            prior_relation='origin' if not prior else 'same' if direction == prior else 'opposite',
            phase_deg=float(np.rad2deg(measured[a] % (2 * np.pi))),
            phase_octant=int((measured[a] % (2 * np.pi)) // (np.pi / 4)),
            pre_move_range_counts=float(np.ptp(quiet) / QUANTUM),
            measured_range_deg=float(np.rad2deg(np.ptp(segment))),
            measured_increment_deg=float(np.rad2deg(measured[b] - measured[a])),
            predicted_increment_deg=float(np.rad2deg(predicted[b] - predicted[a])),
            error_change_deg=float(np.rad2deg(error[b] - error[a])),
            error_at_start_deg=float(np.rad2deg(error[a])),
            error_at_end_deg=float(np.rad2deg(error[b])),
            predicted_start_velocity_rad_s=float(predicted_velocity[a]),
            driven_increment_deg=float(np.rad2deg(measured[e] - measured[a])),
            return_from_extreme_counts=float(direction * (extreme - measured[b]) / QUANTUM)))
    return rows


def hold_windows(counts, telemetry, pre_ms, command_end_ms, bounds, case, split):
    """Nonoverlapping 100ms windows, >=150ms after a reference change.

    Select on measured encoder/reference, never on predicted accuracy. A strong
    contradiction requires every command in the window beyond one static bound.
    """
    position = np.asarray(counts)[::2]
    effort, reference = telemetry[:, 0], telemetry[:, 1]
    changes = np.r_[0, np.flatnonzero(np.diff(reference)) + 1]
    rows = []
    for a in range(pre_ms, command_end_ms - 99, 100):
        b = a + 100
        last_change = changes[changes <= a][-1]
        if a - last_change < 150 or np.ptp(reference[a:b]):
            continue
        # Check all 2kHz encoder samples, not just the 1kHz subset.
        span = int(np.ptp(counts[2*a:2*b]))
        if span > 2:
            continue
        lo, hi = bounds
        u = effort[a:b]
        beyond = bool(np.min(u) > hi or np.max(u) < lo)
        rows.append(dict(source=str(case), split=split, start_ms=a, samples_ms=100,
            reference_counts=int(reference[a]), encoder_span_counts=span,
            position_error_counts=float(reference[a] - np.mean(position[a:b] - position[0])),
            effort_min=int(np.min(u)), effort_max=int(np.max(u)), effort_mean=float(np.mean(u)),
            positive_bound_code=hi, negative_bound_code=lo,
            every_sample_beyond_one_static_bound=beyond,
            static_bound_margin_codes=float(np.min(u) - hi if np.min(u) > hi else
                lo - np.max(u) if np.max(u) < lo else 0)))
    return rows


def group_moves(rows, fields):
    grouped = {}
    for row in rows:
        key = tuple(row[k] for k in fields)
        grouped.setdefault(key, []).append(row)
    result = []
    for key, rr in sorted(grouped.items()):
        error = np.array([r['error_change_deg'] for r in rr])
        motion = np.array([r['measured_range_deg'] for r in rr])
        result.append(dict(zip(fields, key), moves=len(rr),
            captures=len({(r['suite'], r['source']) for r in rr}),
            median_error_change_deg=float(np.median(error)),
            rms_error_change_deg=float(np.sqrt(np.mean(error ** 2))),
            measured_range_min_deg=float(np.min(motion)),
            measured_range_median_deg=float(np.median(motion)),
            measured_range_max_deg=float(np.max(motion))))
    return result


def summarize_driven(windows, fields):
    grouped = {}
    for row in windows:
        key = tuple(row[k] for k in fields)
        move = (row['suite'], row['source'], row['move'])
        grouped.setdefault(key, {}).setdefault(move, []).append(row['conditional_force_ratio'])
    result = []
    for key, moves in sorted(grouped.items()):
        medians = [np.median(values) for values in moves.values()]
        result.append(dict(zip(fields, key), moves=len(moves), windows=sum(map(len, moves.values())),
            median_per_move_force_ratio=float(np.median(medians)),
            min_per_move_force_ratio=float(np.min(medians)), max_per_move_force_ratio=float(np.max(medians))))
    return result


def plot_diagnostics(out, rows, holds, acceleration):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(14, 9), layout='constrained')
    weak = [r for r in rows if r['group'] in ('weak_hold', 'indexed_hold')]
    if len({r['boot_id'] for r in weak}) > 1:
        raise ValueError('phase plot requires a shared encoder boot origin')
    for duration, marker in [(100, 'x'), (200, 'o')]:
        rr = [r for r in weak if r['duration_ms'] == duration]
        axes[0, 0].scatter([r['code']/32767*100 for r in rr],
                         [r['measured_range_deg'] for r in rr], marker=marker,
                         alpha=.65, label=f'{duration} ms')
    axes[0, 0].set(xlabel='Uploaded effort code (% full scale)', ylabel='Measured excursion (degrees)',
                   yscale='symlog', title='Weak commands: small deflections and large launches')
    for code, color in [(-327, 'tab:blue'), (-491, 'tab:orange'), (-655, 'tab:red')]:
        for relation, marker in [('same', 'o'), ('opposite', '^')]:
            rr = [r for r in weak if r['code'] == code and r['duration_ms'] == 200 and r['prior_relation'] == relation]
            if rr:
                axes[0, 1].scatter([r['phase_deg'] for r in rr], [r['measured_range_deg'] for r in rr],
                    color=color, marker=marker, label=f'{code/32767*100:.1f}%, prior {relation}')
    axes[0, 1].set(xlabel='Encoder phase within the same boot (degrees)', ylabel='Measured excursion (degrees)',
                   xlim=(0, 360), yscale='symlog', title='Angle and preparation history remain confounded')
    stationary = [r for r in holds if r['encoder_span_counts'] == 0]
    if stationary:
        lo, hi = stationary[0]['negative_bound_code'], stationary[0]['positive_bound_code']
        axes[1, 0].axhspan(lo, hi, color='0.5', alpha=.18, label='Frozen-v2 static-hold interval')
        for beyond, color, label in [(False, 'tab:blue', 'Not a strict bound violation'),
                                     (True, 'tab:red', 'Every effort sample beyond one bound')]:
            indices = [i for i, r in enumerate(stationary) if r['every_sample_beyond_one_static_bound'] == beyond]
            axes[1, 0].scatter(indices, [stationary[i]['effort_mean'] for i in indices], s=16, color=color, label=label)
    axes[1, 0].set(xlabel='100 ms encoder-stationary windows (not independent)', ylabel='Mean effort code',
                   title='The holding mismatch recurs across controller captures')
    for group in sorted({r['group'] for r in rows}):
        rr = [r for r in rows if r['group'] == group]
        axes[1, 1].scatter([r['measured_range_deg'] for r in rr],
                         [abs(r['error_change_deg']) for r in rr], s=20, alpha=.7, label=group)
    axes[1, 1].set(xlabel='Measured excursion (degrees)', ylabel='Absolute change in prediction error (degrees)',
                   xscale='symlog', yscale='symlog', title='Maneuver diagnostics along continuous replays; no resets')
    for ax in (axes[0, 0], axes[0, 1], axes[1, 1]):
        ax.set_ylim(bottom=0)
    axes[1, 1].set_xlim(left=0)
    for ax in axes.flat:
        ax.grid(alpha=.2)
        ax.legend(fontsize=8)
    fig.suptitle('Frozen command-v2 residual diagnosis · existing recordings only · no plant fit')
    fig.savefig(out / 'residual-diagnostics.png', dpi=150)
    plt.close(fig)
    if acceleration:
        fig, axes = plt.subplots(1, 2, figsize=(12, 4.5), layout='constrained')
        for lo, hi, label, color in [(0, .025, 'Below 2.5% pulses', 'tab:green'),
                (.025, .08, '3–7.5% pulses', 'tab:blue'), (.08, .25, '10–15% pulses', 'tab:purple')]:
            rr = [r for r in acceleration if lo <= abs(r['code'])/32767 < hi]
            axes[0].scatter([abs(r['speed_rad_s']) for r in rr],
                [r['conditional_force_ratio'] for r in rr], s=16, alpha=.55, label=label, color=color)
        per_move = {}
        for r in acceleration:
            key = (r['suite'], r['source'], r['move'], r['code'])
            per_move.setdefault(key, []).append(r['conditional_force_ratio'])
        axes[1].scatter([key[-1]/32767*100 for key in per_move],
                       [np.median(values) for values in per_move.values()], s=24, alpha=.65)
        axes[0].set(xlabel='Measured speed magnitude (rad/s)', ylabel='Conditional force ratio',
                    title='20 ms encoder windows; only already-moving samples')
        axes[0].legend(fontsize=8)
        axes[1].set(xlabel='Uploaded effort code (% full scale)', ylabel='Per-move median conditional force ratio',
                    title='Aggregate within moves before comparing commands')
        for ax in axes:
            ax.axhline(1, color='black', linestyle='--', linewidth=1)
            ax.grid(alpha=.2)
        fig.suptitle('Inferred command force assuming frozen-v2 drag · not measured current or torque')
        fig.savefig(out / 'moving-pulse-gain.png', dpi=150)
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--sources', type=Path, nargs='+', required=True)
    ap.add_argument('--controller', type=Path, nargs='*', default=[])
    ap.add_argument('--model', type=Path, default=ROOT / 'models/axis/sm232ae_tq10_command_v2.json')
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--plot', action='store_true', help='Render diagnostic panels with Matplotlib')
    args = ap.parse_args()
    for paths in (args.sources, args.controller):
        if len({p.resolve() for p in paths}) != len(paths):
            raise ValueError('duplicate input suite')
    out = args.output
    out.mkdir(parents=True, exist_ok=False)
    cfg = read_json(args.model)
    kernel = ROOT / 'examples/axis_command_model_v2.c'
    if sha(kernel) != cfg['kernel_sha256']:
        raise ValueError('kernel differs from frozen model')
    if cfg['encoder_counts_per_rev'] != CPR:
        raise ValueError('unsupported encoder scale')
    p = [cfg['parameters'][n] for n in NAMES]
    model = Model(out / 'kernel')
    inventory = {}
    rows, captures, holds, controller_cases, acceleration = [], [], [], [], []
    for root in args.sources:
        reg = read_json(root / 'registered-before-first-start.json')
        if not read_json(root / 'acquisition-complete.json')['passed']:
            raise ValueError('incomplete suite')
        if reg['model_sha256'] != sha(args.model) or reg['kernel_sha256'] != sha(kernel):
            raise ValueError('registered model/kernel identity mismatch')
        hello = read_json(root / 'health.json')['hello']
        for item in reg['plans']:
            case = root / item['name']
            inventory[str(case)] = verify_effort_capture(case)
            ticks, u, x, dt, plan, run = load_effort(case)
            if dt != .0005 or hello['tick_hz'] != 1000000:
                raise ValueError('requires 2kHz / 1MHz timing')
            if plan['commands'] != item['commands'] or run['command_hash'] != item['command_sha256']:
                raise ValueError('registered waveform mismatch')
            for move in item['moves']:
                if plan['commands'][move['start']:move['start']+move['width']] != [move['code']]*move['width']:
                    raise ValueError('move descriptor differs from registered waveform')
            frozen_file = root / item['prediction_file']
            if sha(frozen_file) != item['prediction_sha256']:
                raise ValueError('registered prediction hash mismatch')
            old = np.loadtxt(frozen_file, delimiter=',', skiprows=1)
            np.testing.assert_array_equal(old[:, 0], ticks - run['command_start_tick'])
            np.testing.assert_array_equal(old[:, 1], u)
            predicted, pv = model.simulate(u[None, :], [len(u)], dt, p)
            predicted, pv = predicted[0], pv[0]
            np.testing.assert_allclose(predicted, old[:, 2], rtol=0, atol=1e-10)
            np.testing.assert_allclose(pv, old[:, 3], rtol=0, atol=1e-10)
            if plan['manifest']['command_period_ticks'] != 20000:
                raise ValueError('targeted move indices require 20ms command intervals')
            if np.ptp(x[:160]) / QUANTUM > 4.000001:
                raise ValueError('nonquiet capture origin')
            t = (ticks - run['command_start_tick']) / hello['tick_hz']
            meta = dict(suite=root.name, source=item['name'], boot_id=hello['boot_id'])
            rows += maneuver_rows(t, x, predicted, pv, item['moves'], meta)
            acceleration += driven_windows(t, x, item['moves'], cfg['parameters'], meta)
            error = predicted - (x - x[0])
            captures.append(dict(**meta, samples=len(x),
                first_tick=int(ticks[0]), last_tick=int(ticks[-1]),
                first_count=int(np.rint(x[0]/QUANTUM)), last_count=int(np.rint(x[-1]/QUANTUM)),
                position_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(error ** 2)))),
                velocity_rmse_rad_s=float(np.sqrt(np.mean((position_velocity(predicted, dt) - position_velocity(x, dt))**2))),
                final_error_deg=float(np.rad2deg(error[-1])),
                registered_prediction_reproduced=True))
            np.savez_compressed(out / (root.name + '--' + item['name'] + '.npz'),
                t=t, measured_rad=x-x[0], predicted_rad=predicted, effort=u)
    verify_phase_chain(captures)
    for root in args.controller:
        train, dev, excluded = select_blocks(root)
        for case, split in [(c, 'training') for c in train] + [(c, 'seen-development') for c in dev]:
            controller_cases.append(str(case))
            _, _, inv = load_servo(case)
            inventory[str(case)] = dict(inv, **{name: dict(sha256=sha(case / name), bytes=(case / name).stat().st_size)
                for name in ('plan.json', 'get_run.json', 'report.json')})
            plan = read_json(case / 'plan.json')['manifest']
            count = np.fromfile(case / 'stream-0.i64le', dtype='<i8')
            telemetry = np.fromfile(case / 'stream-2.i32le', dtype='<i4').reshape(-1, 2)
            pre_ms = plan['capture_pre_roll_ticks'] // 1000
            end_ms = pre_ms + plan['command_sample_count'] * plan['command_period_ticks'] // 1000
            holds += hold_windows(count, telemetry, pre_ms, end_ms, static_bounds(cfg['parameters']), case, split)
    write_csv(out / 'maneuvers.csv', rows)
    write_csv(out / 'captures.csv', captures)
    write_csv(out / 'hold-windows.csv', holds)
    write_csv(out / 'driven-kinematic-windows.csv', acceleration)
    write_csv(out / 'driven-by-command.csv', summarize_driven(acceleration, ['code']))
    write_csv(out / 'driven-by-command-speed.csv', summarize_driven(acceleration, ['code', 'speed_bin_low']))
    for name, fields in [('by-command', ['group', 'code', 'duration_ms']),
                         ('by-history', ['group', 'code', 'duration_ms', 'prior_relation']),
                         ('by-phase', ['boot_id', 'group', 'code', 'duration_ms', 'phase_octant'])]:
        write_csv(out / (name + '.csv'), group_moves(rows, fields))
    summary = dict(captures=len(captures), maneuvers=len(rows),
        all_registered_predictions_reproduced=True, phase_chain_metadata_consistent=True,
        static_bounds_codes=static_bounds(cfg['parameters']),
        controller_captures_checked=len(controller_cases), controller_quiet_windows=len(holds),
        controller_zero_transition_windows=sum(r['encoder_span_counts'] == 0 for r in holds),
        controller_zero_transition_windows_beyond_static_bounds=sum(r['encoder_span_counts'] == 0 and r['every_sample_beyond_one_static_bound'] for r in holds),
        controller_captures_with_zero_transition_violation=len({r['source'] for r in holds if r['encoder_span_counts'] == 0 and r['every_sample_beyond_one_static_bound']}),
        controller_zero_transition_windows_margin_ge_100_codes=sum(r['encoder_span_counts'] == 0 and r['static_bound_margin_codes'] >= 100 for r in holds),
        no_model_fit=True, no_hardware_access=True, no_maneuver_state_resets=True,
        continuous_within_capture=True, assumed_rest_at_capture_origin=True,
        fresh_validation=False, caveats=[
            'Angle and prior motion remain confounded; octants do not establish cogging.',
            'Encoder-stationary does not exclude sub-count motion.',
            'Error changes are differences along continuous replays, not reset predictions.',
            'Commands are not calibrated measured current or torque.',
            'Repeated windows within captures are not independent replicates.',
            'Kinematic force ratios assume incumbent drag; friction and drive-map errors are not separated.'])
    write_json(out / 'summary.json', summary)
    if args.plot:
        plot_diagnostics(out, rows, holds, acceleration)
    source = out / 'source'
    source.mkdir()
    for name in ['axis_residual_diagnostics.py', 'axis_model_fit.py', 'axis_model_fit_v2.py',
                 'axis_fit_controller_workload.py', 'axis_command_model_v2.c']:
        shutil.copy2(ROOT / 'examples' / name, source / name)
    shutil.copy2(args.model, out / 'frozen-model.json')
    write_json(out / 'inventory.json', dict(model_sha256=sha(args.model), inputs=inventory,
        sources={p.name: sha(p) for p in sorted(source.iterdir())},
        registrations={str(root): {name: sha(root / name) for name in
            ['registered-before-first-start.json', 'acquisition-complete.json', 'health.json']}
            for root in args.sources},
        controller_registrations={str(root): {str(path.relative_to(root)): sha(path) for path in
            [root / 'history.json', root / 'registered.json', root / 'transport/health.json']}
            for root in args.controller}))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
