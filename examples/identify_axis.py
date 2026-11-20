#!/usr/bin/env python3
"""Offline single-axis experiment prototype; see docs/axis_identification.md.

NumPy required. Hardware execution is exclusively delegated to an explicit
adapter; the device, not this script, owns realtime scheduling and safety.
"""
import argparse
import csv
import json
import math
from pathlib import Path
import subprocess

import numpy as np


PLAN_FIELDS = ['time', 'segment', 'target', 'excitation']
MEASURE_FIELDS = ['time', 'segment', 'command', 'effort', 'motor_position']


def positive(value):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError('must be finite and positive')
    return value


def write_csv(path, fields, rows):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def make_plan(hz=1000, duration=4, amplitude=.01, effort=.1, low=1, high=50):
    """Each segment starts/ends at rest; dynamic effort has a smooth envelope."""
    rows = []
    tick = 0
    # Smooth out/back traverses, different speeds and acceleration levels.
    specs = [('friction', duration * scale, amplitude) for scale in (2, 1, .5)]
    specs += [('acceleration', duration * scale, amplitude) for scale in (.7, .4)]
    specs += [('dynamic', duration, amplitude), ('validation', duration * .8, amplitude * .73)]
    for segment, length, distance in specs:
        count = max(2, round(length * hz))
        for i in range(count):
            s = i / (count - 1)
            # Quintic rise/fall with dwell at origin and turnaround.
            if s < .1 or s > .9:
                u = 0
            elif s < .4:
                u = (s - .1) / .3
            elif s < .6:
                u = 1
            else:
                u = (.9 - s) / .3
            smooth = u**3 * (10 - 15*u + 6*u*u)
            excitation = 0.0
            if segment == 'dynamic':
                # Hold preloaded target while injecting the chirp; ramps at ends.
                smooth = min(1., s/.1, (1-s)/.1)
                smooth = smooth**3 * (10 - 15*smooth + 6*smooth*smooth)
                t = s * length
                excitation = effort * math.sin(math.pi*s)**2 * math.sin(
                    2*math.pi*(low*t + .5*(high-low)*t*t/length))
            rows.append(dict(time=tick/hz, segment=segment,
                             target=distance*smooth, excitation=excitation))
            tick += 1
    return rows


def read_measurements(path):
    with Path(path).open(newline='') as f:
        reader = csv.DictReader(f)
        if not set(MEASURE_FIELDS).issubset(reader.fieldnames or []):
            raise ValueError('missing measurement columns: ' + ','.join(MEASURE_FIELDS))
        rows = list(reader)
    if len(rows) < 20:
        raise ValueError('need at least 20 samples')
    for row in rows:
        for name in MEASURE_FIELDS:
            if name != 'segment':
                row[name] = float(row[name])
                if not math.isfinite(row[name]):
                    raise ValueError(f'nonfinite {name}')
        if not row['segment']:
            raise ValueError('empty segment')
    if any(b['time'] <= a['time'] for a, b in zip(rows, rows[1:])):
        raise ValueError('time must strictly increase')
    return rows


def demo(rows):
    """Synthetic rigid axis, exact effort recorded, PD tracking plus excitation."""
    x, v = 0., 0.
    previous = float(rows[0]['time'])
    result = []
    for row in rows:
        t = float(row['time'])
        dt = t - previous
        command = 80*(float(row['target'])-x) - 12*v + float(row['excitation'])
        # Integrate held effort; substeps are only for this synthetic fixture.
        for _ in range(10):
            acceleration = (command - .7*v - .015*np.sign(v) - .01) / 2
            v += acceleration * dt/10
            x += v * dt/10
        result.append(dict(time=t, segment=row['segment'], command=command,
                           effort=command, motor_position=x))
        previous = t
    return result


def balances(rows, window, min_speed=1e-4):
    """Integral balance: integral(effort) = M*dv + B*dx + C*integral(sign v) + bias*dt."""
    matrices, efforts, labels = [], [], []
    # Never differentiate or integrate across separate experiment boundaries.
    start = 0
    while start < len(rows):
        end = start + 1
        while end < len(rows) and rows[end]['segment'] == rows[start]['segment']:
            end += 1
        block = rows[start:end]
        if len(block) >= 5:
            t = np.array([r['time'] for r in block])
            x = np.array([r['motor_position'] for r in block])
            u = np.array([r['effort'] for r in block])
            v = np.gradient(x, t, edge_order=2)
            i = 1
            while i < len(t)-2:
                j = int(np.searchsorted(t, t[i]+window))
                if j >= len(t)-1:
                    break
                sl = slice(i, j+1)
                if np.min(np.abs(v[sl])) < min_speed or np.any(np.sign(v[sl]) != np.sign(v[i])):
                    i = j
                    continue
                # Explicit trapezoids support both older and newer NumPy.
                def integral(values):
                    return float(np.sum(.5*(values[:-1]+values[1:])*np.diff(t[sl])))
                matrices.append([v[j]-v[i], x[j]-x[i],
                                 integral(np.sign(v[sl])), t[j]-t[i]])
                efforts.append(integral(u[sl]))
                labels.append(block[0]['segment'])
                i = j
        start = end
    return np.array(matrices).reshape((-1, 4)), np.array(efforts), np.array(labels)


def fit(rows, window=.02, min_speed=1e-4):
    a, b, labels = balances(rows, window, min_speed)
    train = np.isin(labels, ['friction', 'acceleration'])
    valid = labels == 'validation'
    if train.sum() < 8 or valid.sum() < 2:
        raise ValueError('need friction/acceleration training and held-out validation windows')
    scale = np.linalg.norm(a[train], axis=0)
    if np.any(scale == 0):
        raise ValueError('unexcited parameter: zero regression column')
    scaled = a[train] / scale
    theta, _, rank, singular = np.linalg.lstsq(scaled, b[train], rcond=None)
    if rank < 4:
        raise ValueError('parameters not identifiable: rank-deficient regression')
    theta /= scale
    if theta[0] <= 0 or np.any(theta[1:3] < 0):
        raise ValueError('nonphysical baseline fit; inspect excitation, units, and residuals')
    report = dict(model='rigid_integral_balance',
                  parameters=dict(zip(['inertia', 'viscous', 'coulomb', 'bias'], theta.tolist())),
                  scaled_condition=float(singular[0]/singular[-1]), window_seconds=window,
                  min_speed=min_speed,
                  warnings=['Preliminary OLS fit; feedback/noise can bias parameters.',
                            'Validation is effort balance, NOT load trajectory prediction.'])
    if report['scaled_condition'] > 1000:
        report['warnings'].append('Poor conditioning: parameters may not be distinguishable.')
    for name, mask in [('training', train), ('validation', valid)]:
        residual = b[mask]-a[mask]@theta
        report[name] = dict(windows=int(mask.sum()),
                            effort_integral_rmse=float(np.sqrt(np.mean(residual**2))),
                            relative_error=float(np.linalg.norm(residual)/max(np.linalg.norm(b[mask]), 1e-15)))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    for name in ('plan', 'demo'):
        p = sub.add_parser(name)
        p.add_argument('--output', type=Path, required=True)
        p.add_argument('--hz', type=positive, default=1000)
        p.add_argument('--duration', type=positive, default=4)
        p.add_argument('--amplitude', type=positive, default=.01)
        p.add_argument('--effort', type=positive, default=.1)
        p.add_argument('--low', type=positive, default=1)
        p.add_argument('--high', type=positive, default=50)
    p = sub.add_parser('fit')
    p.add_argument('input', type=Path)
    p.add_argument('--window', type=positive, default=.02)
    p.add_argument('--min-speed', type=positive, default=1e-4,
                   help='exclude near-zero/reversing windows; position units per second')
    p.add_argument('--output', type=Path, required=True)
    p = sub.add_parser('capture')
    p.add_argument('--plan', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--adapter', type=Path, required=True)
    args = parser.parse_args()
    try:
        if args.action in ('plan', 'demo'):
            if args.high <= args.low or args.high >= args.hz/10:
                raise ValueError('require low < high < hz/10')
            rows = make_plan(args.hz, args.duration, args.amplitude, args.effort, args.low, args.high)
            write_csv(args.output, PLAN_FIELDS if args.action == 'plan' else MEASURE_FIELDS,
                      rows if args.action == 'plan' else demo(rows))
        elif args.action == 'capture':
            if args.output.exists():
                raise ValueError('refusing to overwrite existing acquisition')
            if not args.plan.is_file():
                raise ValueError('plan does not exist')
            args.output.parent.mkdir(parents=True, exist_ok=True)
            subprocess.run([str(args.adapter.resolve()), '--plan', str(args.plan.resolve()),
                            '--output', str(args.output.resolve())], check=True)
            read_measurements(args.output)
        else:
            report = fit(read_measurements(args.input), args.window, args.min_speed)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    except (ValueError, OSError, subprocess.CalledProcessError) as exc:
        parser.exit(1, f'error: {exc}\n')


if __name__ == '__main__':
    main()
