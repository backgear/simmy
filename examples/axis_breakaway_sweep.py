#!/usr/bin/env python3
"""Explicitly authorized, bounded torque-ramp breakaway sweep on the bench rig.

Dry-run by default. Same authorized-envelope convention as axis_pulse_trials.py:
bounded command ceiling, travel guard, no calibrated effort or physical inertia
claim. Unlike that script's short rectangular pulses, this issues one slow
triangular ramp (0 -> target -> 0) per trial, at two ramp rates and both
directions, and reports the smallest command code at which the encoder first
moves -- the empirical breakaway torque -- rather than inferring it from a
coast-down fit.

Like axis_pulse_trials.py, this never asserts output_enable_used (the device
always drives PIN_ENABLE low at ARM); if the bench's amplifier enable is
MCU-controlled rather than a fixed jumper, no real motion will occur and every
trial will report no breakaway, which is the safe failure direction.
"""
import argparse
import json
from pathlib import Path
import struct

import axis_recorder as recorder


def ramp_values(target_code, half=299):
    """0..target..0, zero at both ends, (2*half-1) samples."""
    up = [round(target_code * i / half) for i in range(half + 1)]
    down = [round(target_code * (half - i) / half) for i in range(1, half + 1)]
    return up + down


def trial_plan(target_code, rates_s, directions=(1, -1)):
    if not 0 < target_code <= 3276:
        raise ValueError('authorized envelope: 0 < target_code <= 3276')
    trials = []
    for rate_s in rates_s:
        if not 1 <= rate_s <= 60:
            raise ValueError('authorized envelope: 1 <= ramp rate <= 60 seconds')
        for direction in directions:
            values = ramp_values(direction * target_code)
            trials.append(dict(name=f'ramp-{rate_s}s-' + ('positive' if direction > 0 else 'negative'),
                               direction=direction, rate_s=rate_s, duration_s=2 * rate_s, commands=values))
    return trials


def find_breakaway(directory, stream_id=0, noise_counts=4, hold_samples=5):
    """First sustained encoder departure from rest, and the command code at
    that tick. Returns None if the encoder never moved beyond noise."""
    directory = Path(directory)
    plan = json.loads((directory / 'plan.json').read_text())
    run = json.loads((directory / 'get_run.json').read_text())
    health = json.loads((directory.parent / 'health.json').read_text())
    hz = health['hello']['tick_hz']
    stream = next(s for s in run['streams'] if s['stream_id'] == stream_id)
    raw = (directory / f'stream-{stream_id}.i64le').read_bytes()
    counts = struct.unpack('<' + 'q' * stream['sample_count'], raw)
    period = stream['period_ticks']
    manifest = plan['manifest']
    cp = manifest['command_period_ticks']
    commands = plan['commands']
    rest = counts[0]
    moved_at = None
    for i in range(len(counts)):
        if abs(counts[i] - rest) > noise_counts:
            window = counts[i:i + hold_samples]
            if len(window) == hold_samples and all(abs(c - rest) > noise_counts for c in window):
                moved_at = i
                break
    if moved_at is None:
        return dict(breakaway_code=None, breakaway_time_s=None, max_excursion_counts=max(abs(c - rest) for c in counts))
    tick = stream['first_sample_tick'] + moved_at * period
    command_index = (tick - run['command_start_tick']) // cp
    code = commands[command_index] if 0 <= command_index < len(commands) else None
    return dict(breakaway_code=code, breakaway_time_s=(tick - run['command_start_tick']) / hz,
               breakaway_encoder_delta=counts[moved_at] - rest)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target-code', type=int, default=3276)
    parser.add_argument('--rates-s', type=float, nargs='+', default=[5, 15])
    parser.add_argument('--travel-counts', type=int, required=True)
    parser.add_argument('--execute-powered', action='store_true')
    args = parser.parse_args()
    trials = trial_plan(args.target_code, args.rates_s)
    if not 0 < args.travel_counts <= 16384:
        raise ValueError('travel cutoff must be 1..16384 counts; this is not active braking')
    config = dict(trials=[{k: v for k, v in t.items() if k != 'commands'} for t in trials],
                  target_code=args.target_code, counts_per_revolution=4096, analog_enabled=False,
                  output_enable_used=False, input_kind='command, NOT measured effort')
    if not args.execute_powered:
        print(json.dumps(config, indent=2))
        return
    t = recorder.Transport(args.port, args.output)
    completed = []
    failed = False
    try:
        hello = recorder.health(t)
        if hello['tick_hz'] != 1000000:
            raise ValueError('trial timing currently requires a 1 MHz device clock')
        recorder.save(args.output / 'experiment.json', config)
        for trial in trials:
            print('RUN', trial['name'], flush=True)
            capture = recorder.record(t, hello, trial['name'], [0], False, 500, trial['duration_s'],
                                      command_values=trial['commands'],
                                      post_roll_ticks=hello['tick_hz'],
                                      travel_limit_counts=args.travel_counts)
            entry = dict(name=trial['name'], capture=capture)
            completed.append(entry)
            if not capture['passed']:
                failed = True
            else:
                entry['breakaway'] = find_breakaway(args.output / trial['name'])
            recorder.save(args.output / 'trial-summary.json', completed)
            print(json.dumps(entry), flush=True)
            if failed:
                print('STOP: no further trials', flush=True)
                break
    finally:
        t.close()
    if failed:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
