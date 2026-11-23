#!/usr/bin/env python3
"""Report recorded single-polarity effort-command pulses, not measured torque.

Reads recorder case directories; never connects to hardware. Position is relative
only for display; original absolute counts/ticks remain in motion.csv and source
bundles. Velocity is a centered finite difference with a +/-5 ms window, clipped
at endpoints. No fitting, time shifting, or effort substitution is performed.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import struct


def summarize(directory, counts_per_rev=4096, stream_id=0):
    directory = Path(directory)
    if counts_per_rev <= 0:
        raise ValueError('counts_per_rev must be positive')
    plan = json.loads((directory / 'plan.json').read_text())
    run = json.loads((directory / 'get_run.json').read_text())
    report = json.loads((directory / 'report.json').read_text())
    health = json.loads((directory.parent / 'health.json').read_text())
    if not report['passed'] or report.get('state') != 'COMPLETE':
        raise ValueError('only complete, integrity-checked pulse runs can be summarized')
    hz = health['hello']['tick_hz']
    stream = next(s for s in run['streams'] if s['stream_id'] == stream_id)
    raw = (directory / f'stream-{stream_id}.i64le').read_bytes()
    if len(raw) != stream['sample_count'] * 8 or stream['channels'] != 1:
        raise ValueError('encoder raw byte count/shape mismatch')
    counts = struct.unpack('<' + 'q' * stream['sample_count'], raw)
    if len(counts) < 2:
        raise ValueError('too few samples')
    manifest = plan['manifest']
    period = stream['period_ticks']
    commands = plan['commands']
    active = [i for i, v in enumerate(commands) if v]
    if not active or any(i not in active for i in range(active[0], active[-1]+1)):
        raise ValueError('expected one contiguous nonzero pulse')
    signs = {1 if commands[i] > 0 else -1 for i in active}
    if len(signs) != 1:
        raise ValueError('expected single-polarity pulse')
    direction = signs.pop()
    cp = manifest['command_period_ticks']
    pulse_start = active[0] * cp / hz
    pulse_end = (active[-1] + 1) * cp / hz
    ticks = [stream['first_sample_tick'] + i * period for i in range(len(counts))]
    times = [(tick - run['command_start_tick']) / hz for tick in ticks]
    deg = [(v - counts[0]) * 360 / counts_per_rev for v in counts]
    window = max(1, round(.005 * hz / period))
    velocity = []
    rows = []
    for i, t in enumerate(times):
        lo, hi = max(0, i-window), min(len(counts)-1, i+window)
        v = (deg[hi] - deg[lo]) / (times[hi] - times[lo])
        velocity.append(v)
        index = (ticks[i] - run['command_start_tick']) // cp
        code = commands[index] if 0 <= index < len(commands) else 0
        segment = ('pre_roll' if index < 0 else 'post_roll' if index >= len(commands)
                   else 'pulse' if code else 'zero')
        rows.append(dict(tick=ticks[i], time_s=t, segment=segment, command_code=code,
                         command_fraction=code / manifest['analog_output_full_scale'],
                         encoder_count=counts[i], position_rev=(counts[i]-counts[0])/counts_per_rev,
                         position_deg=deg[i], velocity_deg_s=v))
    tail = [v for t, v in zip(times, counts) if t >= times[-1]-.2]
    peak_index = max(range(len(counts)), key=lambda i: direction*(counts[i]-counts[0]))
    last_outside = max((i for i, v in enumerate(counts) if abs(v-counts[-1]) > 2), default=-1)
    settle_index = max(last_outside+1, math.ceil((pulse_end-times[0])*hz/period))
    stationary = len(tail) > 1 and max(tail)-min(tail) <= 2
    summary = dict(run_id=plan['run_id'], direction=direction, counts_per_rev=counts_per_rev,
                   command_fraction=commands[active[0]]/manifest['analog_output_full_scale'],
                   pulse_start_s=pulse_start, pulse_duration_s=pulse_end-pulse_start,
                   initial_count=counts[0], final_count=counts[-1],
                   initial_phase_deg_arbitrary_zero=(counts[0] % counts_per_rev)*360/counts_per_rev,
                   displacement_counts=counts[-1]-counts[0], displacement_deg=deg[-1],
                   peak_displacement_deg=deg[peak_index], peak_time_s=times[peak_index],
                   peak_velocity_deg_s=velocity[max(range(len(velocity)), key=lambda i: direction*velocity[i])],
                   final_200ms_range_counts=max(tail)-min(tail), stationary_tail=stationary,
                   settling_time_after_pulse_s=(max(0, times[settle_index]-pulse_end)
                                               if stationary and settle_index < len(times) else None),
                   settling_criterion='within +/-2 counts of final value for remainder of recording',
                   velocity_method='centered +/-5ms finite difference, endpoint clipped',
                   input_kind='uploaded command, NOT measured effort', source=str(directory))
    with (directory / 'motion.csv').open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    (directory / 'motion-summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    return summary, rows


def plot_results(results, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    for summary, rows in results:
        label = f"{100*summary['command_fraction']:+.1f}% {Path(summary['source']).name}"
        t = [r['time_s'] for r in rows]
        for axis, key in zip(axes, ('position_deg', 'velocity_deg_s', 'command_fraction')):
            axis.plot(t, [r[key] for r in rows], label=label)
    for axis, ylabel in zip(axes, ('Position relative to capture start (deg)',
                                    'Velocity (deg/s; 10 ms difference)',
                                    'Command / full scale (not effort)')):
        axis.set_ylabel(ylabel)
        axis.grid(True, alpha=.3)
    axes[0].legend(fontsize='small', ncol=2)
    axes[-1].set_xlabel('Time relative to command start (s)')
    fig.suptitle('Recorded pulse responses — no fitted time or amplitude adjustments')
    fig.tight_layout()
    fig.savefig(output)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('cases', type=Path, nargs='+')
    parser.add_argument('--counts-per-rev', type=int, default=4096)
    parser.add_argument('--plot', type=Path)
    args = parser.parse_args()
    results = [summarize(path, args.counts_per_rev) for path in args.cases]
    if args.plot:
        plot_results(results, args.plot)
    print(json.dumps([s for s, rows in results], indent=2))


if __name__ == '__main__':
    main()
