#!/usr/bin/env python3
"""Batched bare-shaft phase probes. Dry-run by default; ADC remains disabled.

Small profile: +/-5% for 5ms probes, +10% for 20ms indexing.
Large profile: six alternating +/-10% / 20ms probes, 550ms start spacing.
2kHz capture, 1s post-roll, two-revolution command-to-zero excursion guard.
The guard is not a brake and cannot disable the drive-enable jumper.
"""
import argparse
import csv
import json
from pathlib import Path
import statistics
import struct

import axis_recorder as recorder


def waveform(profile='small', batch=1):
    values = [0]*600  # 3 seconds, 5ms command grid
    legs = []
    if profile == 'small':
        pulses = ((30, 1638, 1, 'probe'), (100, -1638, 1, 'probe'),
                  (190, 3276, 4, 'index'), (300, -1638, 1, 'probe'),
                  (370, 1638, 1, 'probe'), (460, 3276, 4, 'index'))
    elif profile == 'large':
        initial = 1 if batch % 2 else -1
        pulses = [(20+110*i, initial*(-1)**i*3276, 4, 'probe') for i in range(6)]
    else:
        raise ValueError('unknown probe profile')
    for index, (start, code, length, role) in enumerate(pulses):
        values[start:start+length] = [code]*length
        legs.append(dict(leg=index, start_sample=start, end_sample=start+length,
                         code=code, role=role,
                         site_within_batch=int(index >= 3) if profile == 'small' else index))
    return values, legs


def analyze(case, batch):
    case = Path(case)
    health = json.loads((case.parent/'health.json').read_text())
    run = json.loads((case/'get_run.json').read_text())
    plan = json.loads((case/'plan.json').read_text())
    capture = json.loads((case/'report.json').read_text())
    if not capture['passed'] or capture['state'] != 'COMPLETE':
        raise ValueError('incomplete or invalid phase recording')
    desc = next(s for s in run['streams'] if s['stream_id'] == 0)
    data = (case/'stream-0.i64le').read_bytes()
    if len(data) != desc['sample_count']*8:
        raise ValueError('encoder sample count mismatch')
    counts = struct.unpack('<'+'q'*desc['sample_count'], data)
    hz = health['hello']['tick_hz']
    cp = plan['manifest']['command_period_ticks']
    period = desc['period_ticks']
    first = desc['first_sample_tick']
    origin = run['command_start_tick']
    experiment_path = case.parent/'experiment.json'
    experiment = json.loads(experiment_path.read_text()) if experiment_path.exists() else {}
    profile = experiment.get('profile', 'small')
    values, legs = waveform(profile, batch)
    if plan['commands'] != values or cp != 5000:
        raise ValueError('unexpected phase-probe waveform/timing')

    def window(start, end):
        lo = max(0, (start-first+period-1)//period)
        hi = min(len(counts), (end-first)//period+1)
        if hi <= lo:
            raise ValueError('missing analysis window')
        return counts[lo:hi]

    metrics = []
    for leg in legs:
        start = origin+leg['start_sample']*cp
        end = origin+leg['end_sample']*cp
        pre = window(start-75000, start-25000)
        endpoint_start, endpoint_end = ((200000, 250000) if profile == 'small'
                                        else (450000, 500000))
        post = window(end+endpoint_start, end+endpoint_end)
        response = window(start, end+endpoint_end)
        a, b = statistics.median(pre), statistics.median(post)
        direction = 1 if leg['code'] > 0 else -1
        peak = max(response) if direction > 0 else min(response)
        metric = dict(batch=batch, leg=leg['leg'], role=leg['role'],
                      profile=profile,
                      site=(batch-1)*(2 if profile == 'small' else 6)+leg['site_within_batch'],
                      order=(('positive-first' if leg['site_within_batch'] == 0 else 'negative-first')
                             if profile == 'small' else
                             ('positive-first' if batch % 2 else 'negative-first')),
                      pulse_duration_ms=(end-start)*1000/hz,
                      command_start_tick=start, command_end_tick=end,
                      command_code=leg['code'], direction=direction,
                      initial_count=a, final_count=b,
                      initial_phase_deg_arbitrary_zero=(a % 4096)*360/4096,
                      displacement_counts=b-a, displacement_deg=(b-a)*360/4096,
                      peak_displacement_deg=(peak-a)*360/4096,
                      pre_zero_range_counts=max(pre)-min(pre),
                      post_zero_range_counts=max(post)-min(post),
                      stationary_windows=max(pre)-min(pre) <= 2 and max(post)-min(post) <= 2)
        metrics.append(metric)
    write_csv(case/'pulse-metrics.csv', metrics)
    return metrics


def write_csv(path, rows):
    with Path(path).open('w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def plot(root, rows):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    for axis, direction in zip(axes, (1, -1)):
        for order, marker in (('positive-first', 'o'), ('negative-first', 's')):
            mm = [m for m in rows if m['role'] == 'probe' and m['direction'] == direction and m['order'] == order]
            axis.scatter([m['initial_phase_deg_arbitrary_zero'] for m in mm],
                         [m['displacement_deg'] for m in mm], marker=marker, label=order)
        example = next(m for m in rows if m['role'] == 'probe' and m['direction'] == direction)
        axis.set_ylabel(f"{example['command_code']/32767*100:+.1f}% / {example['pulse_duration_ms']:g}ms net move (deg)")
        axis.grid(True, alpha=.3)
        axis.legend()
    axes[-1].set_xlabel('Starting encoder phase modulo one revolution (deg; arbitrary zero)')
    fig.suptitle('Pulse response versus shaft phase — no fitted correction')
    fig.tight_layout()
    fig.savefig(root/'phase-probe-response.svg')
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--batches', type=int, default=8)
    parser.add_argument('--profile', choices=('small', 'large'), default='small')
    parser.add_argument('--execute-powered', action='store_true')
    parser.add_argument('--analyze-only', action='store_true')
    args = parser.parse_args()
    if not 1 <= args.batches <= 12:
        raise ValueError('1..12 batches permitted')
    values, legs = waveform(args.profile)
    experiment = dict(profile=args.profile, command_period_ticks=5000, duration_seconds=3,
                      first_batch_commands=values,
                      waveform_note='large profile alternates starting sign between batches',
                      legs=legs, batches=args.batches, capture_period_ticks=500,
                      travel_limit_counts=8192, post_roll_ticks=1000000,
                      input_kind='command, NOT measured torque',
                      endpoint_window=('200..250ms after pulse end' if args.profile == 'small'
                                       else '450..500ms after pulse end'),
                      baseline_window='75..25ms before pulse start')
    all_metrics = []
    if args.analyze_only:
        for case in sorted(args.output.glob('batch-*')):
            all_metrics.extend(analyze(case, int(case.name.split('-')[1])))
    elif not args.execute_powered:
        print(json.dumps(experiment, indent=2))
        return
    else:
        if not args.port:
            raise ValueError('--port required for powered execution')
        t = recorder.Transport(args.port, args.output)
        try:
            hello = recorder.health(t)
            if hello['tick_hz'] != 1000000:
                raise ValueError('requires 1MHz tick')
            recorder.save(args.output/'experiment.json', experiment)
            for batch in range(1, args.batches+1):
                name = f'batch-{batch:02d}'
                batch_values, _ = waveform(args.profile, batch)
                capture = recorder.record(t, hello, name, [0], False, 500, 3,
                                          command_values=batch_values, post_roll_ticks=1000000,
                                          travel_limit_counts=8192)
                print('CAPTURE', name, json.dumps(capture), flush=True)
                if not capture['passed']:
                    print('STOP: capture fault/integrity failure', flush=True)
                    raise SystemExit(1)
                metrics = analyze(args.output/name, batch)
                all_metrics.extend(metrics)
                write_csv(args.output/'pulse-metrics.csv', all_metrics)
                print('PROBES', json.dumps([m for m in metrics if m['role'] == 'probe']), flush=True)
                if not all(m['stationary_windows'] for m in metrics):
                    print('STOP: nonstationary analysis window; no further batches', flush=True)
                    raise SystemExit(1)
        finally:
            t.close()
    if all_metrics:
        write_csv(args.output/'pulse-metrics.csv', all_metrics)
        plot(args.output, all_metrics)
        probes = [m for m in all_metrics if m['role'] == 'probe']
        summary = dict(pulses=len(all_metrics), probes=len(probes),
                       sites=len({m['site'] for m in probes}),
                       all_windows_stationary=all(m['stationary_windows'] for m in all_metrics),
                       maximum_pre_zero_range_counts=max(m['pre_zero_range_counts'] for m in all_metrics),
                       maximum_post_zero_range_counts=max(m['post_zero_range_counts'] for m in all_metrics))
        recorder.save(args.output/'phase-summary.json', summary)
        print('SUMMARY', json.dumps(summary), flush=True)


if __name__ == '__main__':
    main()
