#!/usr/bin/env python3
"""Parallel independent BLDC C trials, deterministic CSV order and atomic output.
Offline fixed-step, no cross-trial barrier; default 256 workers, limited
only by trial count. Python stdlib only; no simulation equations here.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor, as_completed
import csv
import io
import math
from pathlib import Path
import subprocess
import sys

from run_stepper_sweep import write_atomic, default_jobs


def trial(binary, name, steps):
    result = subprocess.run([str(binary), "--case", name, "--steps", str(steps)],
                            capture_output=True, text=True, check=False)
    if result.returncode:
        raise RuntimeError(f"{name} failed ({result.returncode}): {result.stderr.strip()}")
    rows = list(csv.reader(io.StringIO(result.stdout)))
    if len(rows) < 2 or not rows[0] or rows[0][0] != "case":
        raise RuntimeError(f"invalid CSV from {name}")
    for row in rows[1:]:
        if len(row) != len(rows[0]) or row[0] != name:
            raise RuntimeError(f"invalid CSV row from {name}")
        try:
            if not all(math.isfinite(float(value)) for value in row[1:]):
                raise ValueError("nonfinite")
        except ValueError as error:
            raise RuntimeError(f"invalid numeric CSV from {name}") from error
    return rows[0], rows[1:]


def run_sweep(binary, names, steps, jobs):
    if not names or jobs < 1 or steps < 1:
        raise ValueError("nonempty cases and positive jobs/steps required")
    ordered = [None] * len(names)
    with ThreadPoolExecutor(max_workers=min(jobs, len(names))) as pool:
        futures = {pool.submit(trial, binary, name, steps): i for i, name in enumerate(names)}
        try:
            for future in as_completed(futures):
                ordered[futures[future]] = future.result()
        except BaseException:
            for future in futures:
                future.cancel()
            raise
    header = ordered[0][0]
    if any(item[0] != header for item in ordered):
        raise RuntimeError("inconsistent CSV headers")
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(header)
    for _, rows in ordered:
        writer.writerows(rows)
    return output.getvalue()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, default=Path(__file__).resolve().parents[1]/"build/bldc_faults")
    parser.add_argument("--jobs", type=int, default=default_jobs(), help="default 256, limited by trial count")
    parser.add_argument("--steps", type=int, default=4, help="plant substeps per fixed 5 us PWM quantum; default 4")
    parser.add_argument("--case", action="append", dest="cases", help="subset; repeat for multiple cases")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.jobs < 1 or not 1 <= args.steps <= 1000:
        parser.error("jobs >=1, steps 1..1000")
    try:
        binary = args.binary.resolve()
        result = subprocess.run([str(binary), "--list"], capture_output=True, text=True, check=True)
        available = result.stdout.splitlines()
        names = args.cases or available
        if not available or len(set(available)) != len(available) or any(n not in available for n in names):
            raise RuntimeError("empty/duplicate case list or unknown requested case")
        if len(set(names)) != len(names):
            raise RuntimeError("duplicate requested case")
        output = run_sweep(binary, names, args.steps, args.jobs)
        if args.output:
            write_atomic(args.output, output)
        else:
            sys.stdout.write(output)
    except (OSError, subprocess.CalledProcessError, RuntimeError) as error:
        print(f"BLDC sweep failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
