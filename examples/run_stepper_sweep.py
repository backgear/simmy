#!/usr/bin/env python3
"""Run independent C --point trials concurrently, merging CSV in speed order.
Python standard library only; the C-only serial --sweep remains available.
Trials advance fixed simulated-time steps, not wall-clock/realtime steps.
There is no cross-trial lockstep barrier; OS scheduling changes runtime,
not simulated dt. Default: 256 workers, limited by available trials.
No simulation logic lives here. Example:
  python3 examples/run_stepper_sweep.py compensation \
      --output build/stepper_compensation_sweep.csv
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import io
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def default_jobs():
    # Trials have small memory footprints and variable durations. Permit
    # overcommit instead of imposing a CPU-count ceiling; --jobs overrides.
    return 256


def point(binary, speed, hz):
    # Threads only supervise independent CPU-bound C subprocesses; no Python
    # GIL bottleneck and no shared simulation/RNG state between trials.
    result = subprocess.run([str(binary), "--point", repr(speed), str(hz)],
                            capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError(f"trial at {speed:.9g} rad/s failed ({result.returncode}): "
                           f"{result.stderr.strip()}")
    rows = list(csv.reader(io.StringIO(result.stdout)))
    if len(rows) < 2 or not rows[0] or any(len(row) != len(rows[0]) for row in rows[1:]):
        raise RuntimeError(f"invalid CSV from trial at {speed:.9g} rad/s")
    return rows[0], rows[1:]


def run_sweep(binary, speeds, hz, jobs):
    """Return complete validated output, never a partial merged CSV."""
    ordered = [None] * len(speeds)
    with ThreadPoolExecutor(max_workers=min(jobs, len(speeds))) as pool:
        futures = {pool.submit(point, binary, speed, hz): i for i, speed in enumerate(speeds)}
        try:
            for future in as_completed(futures):
                ordered[futures[future]] = future.result()
        except BaseException:
            for future in futures:
                future.cancel()
            raise
    header = ordered[0][0]
    if any(item[0] != header for item in ordered):
        raise RuntimeError("inconsistent CSV headers across trials")
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(header)
    for _, rows in ordered:
        writer.writerows(rows)
    return output.getvalue()


def write_atomic(path, text):
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, encoding="utf-8",
                                         newline="", delete=False) as f:
            temporary = Path(f.name)
            f.write(text)
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("experiment", choices=("velocity", "compensation"))
    parser.add_argument("--jobs", type=int, default=default_jobs(),
                        help="default: 256, capped by number of trials")
    parser.add_argument("--points", type=int, help="default: velocity 121, compensation 61")
    parser.add_argument("--min-speed", type=float, default=.01)
    parser.add_argument("--max-speed", type=float, default=1.2)
    parser.add_argument("--hz", type=int, default=10000)
    parser.add_argument("--binary", type=Path, help="override C executable path")
    parser.add_argument("--output", type=Path, help="atomic file output; default: stdout")
    args = parser.parse_args()
    points = args.points if args.points is not None else (121 if args.experiment == "velocity" else 61)
    if not (math.isfinite(args.min_speed) and math.isfinite(args.max_speed) and
            .005 <= args.min_speed <= args.max_speed <= 2):
        parser.error("speeds must be finite and .005 <= min <= max <= 2 rad/s")
    if not (2 <= points <= 1000 and args.jobs >= 1 and 1000 <= args.hz <= 100000):
        parser.error("points: 2..1000; jobs: >=1; hz: 1000..100000")
    binary = args.binary or (Path(__file__).resolve().parents[1] / "build" / f"stepper_{args.experiment}")
    binary = binary.resolve()
    speeds = [args.min_speed * math.pow(args.max_speed / args.min_speed, i / (points-1))
              for i in range(points)]
    try:
        output = run_sweep(binary, speeds, args.hz, args.jobs)
        if args.output:
            write_atomic(args.output, output)
        else:
            sys.stdout.write(output)
    except (OSError, RuntimeError) as error:
        print(f"sweep failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
