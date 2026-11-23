#!/usr/bin/env python3
"""Explicitly authorized, bounded alternating pulse trials on the bench rig.

Dry-run by default. This is not a universal safe machine configuration. Assumes
manually enabled bare shaft, encoder 0 on PA0/PA1, command PWM on PA6/PA7,
4096 counted edges/rev. ADC and MCU enable assertion are disabled. Travel guard
commands zero on violation; it cannot brake or disable a jumper-enabled drive.
No calibrated effort or physical inertia is inferred from command codes.
"""
import argparse
import json
from pathlib import Path

import axis_recorder as recorder
import axis_pulse_report as report


def trial_plan(amplitude_percent, pulse_ms, pairs):
    if not 0 < amplitude_percent <= 10 or not 1 <= pulse_ms <= 20 or not 1 <= pairs <= 3:
        raise ValueError('authorized envelope: 0 < amplitude <= 10%, 1..20 ms, 1..3 pairs')
    # Truncate instead of rounding above the requested full-scale ceiling.
    code = int(32767 * amplitude_percent / 100)
    if code == 0:
        raise ValueError('amplitude below integer-code resolution')
    trials = []
    for pair in range(1, pairs+1):
        for direction in (1, -1):
            name = f'pair-{pair}-' + ('positive' if direction > 0 else 'negative')
            values = [0]*20 + [code*direction]*pulse_ms + [0]*(80-pulse_ms)
            trials.append(dict(name=name, direction=direction, commands=values))
    return trials


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--amplitude-percent', type=float, required=True)
    parser.add_argument('--pulse-ms', type=int, required=True)
    parser.add_argument('--pairs', type=int, required=True)
    parser.add_argument('--travel-counts', type=int, required=True)
    parser.add_argument('--execute-powered', action='store_true')
    args = parser.parse_args()
    trials = trial_plan(args.amplitude_percent, args.pulse_ms, args.pairs)
    if not 0 < args.travel_counts <= 8192:
        raise ValueError('travel cutoff must be 1..8192 counts; this is not active braking')
    config = dict(trials=trials, capture_period_ticks=500, command_period_ticks=1000,
                  capture_post_roll_ticks=1000000, travel_limit_counts=args.travel_counts,
                  counts_per_revolution=4096, analog_enabled=False,
                  output_enable_used=False, input_kind='command, NOT measured effort')
    if not args.execute_powered:
        print(json.dumps(config, indent=2))
        return
    t = recorder.Transport(args.port, args.output)
    completed = []
    plots = []
    failed = False
    try:
        hello = recorder.health(t)
        if hello['tick_hz'] != 1000000:
            raise ValueError('trial timing currently requires a 1 MHz device clock')
        recorder.save(args.output / 'experiment.json', config)
        for trial in trials:
            print('RUN', trial['name'], flush=True)
            capture = recorder.record(t, hello, trial['name'], [0], False, 500, .1,
                                      command_values=trial['commands'],
                                      post_roll_ticks=hello['tick_hz'],
                                      travel_limit_counts=args.travel_counts)
            entry = dict(name=trial['name'], capture=capture)
            completed.append(entry)
            if not capture['passed']:
                failed = True
            else:
                summary, rows = report.summarize(args.output / trial['name'])
                entry['motion'] = summary
                plots.append((summary, rows))
                if not summary['stationary_tail']:
                    entry['stop_reason'] = 'no stationary final 200 ms tail within 2 counts'
                    failed = True
                elif summary['displacement_counts'] * trial['direction'] < -2:
                    entry['stop_reason'] = 'motion sign disagrees with command sign'
                    failed = True
            recorder.save(args.output / 'trial-summary.json', completed)
            print(json.dumps(entry), flush=True)
            if failed:
                print('STOP: no further pulses', flush=True)
                break
        if plots:
            report.plot_results(plots, args.output / 'pulse-comparison.svg')
    finally:
        t.close()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
