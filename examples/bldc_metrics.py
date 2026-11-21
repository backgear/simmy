"""Stdlib-only BLDC experiment scoring; policy belongs here, not the plant.
Heat is modeled ELECTRICAL dissipation, not temperature or complete losses.
Cumulative integrals avoid bias from mixed-rate overview/waveform CSV rows.
"""
import math


def window_stats(rows, start=.5, end=1.5):
    if end <= start:
        raise ValueError("positive window duration required")
    boundary = lambda time: next((r for r in rows if abs(r["time_s"]-time) < 1e-9), None)
    a, b = boundary(start), boundary(end)
    if a is None or b is None:
        raise ValueError("window boundaries missing")
    difference = lambda field: b[field]-a[field]
    heat, work, error = difference("heat_j"), difference("load_work_j"), difference("error_square_integral")
    if not all(math.isfinite(v) and v >= 0 for v in (heat, work, error)):
        raise ValueError("invalid cumulative heat/work/tracking integrals")
    result = dict(window_start_s=start, window_end_s=end, heat_j=heat, load_work_j=work,
                  tracking_rms_rad_s=math.sqrt(error/(end-start)),
                  numerical_loss_j=difference("numerical_loss_j"),
                  diode_heat_j=difference("diode_heat_j"), switch_heat_j=difference("switch_heat_j"))
    for field in ["winding_a_heat_j", "winding_b_heat_j", "winding_c_heat_j", *[f"fet{j}_heat_j" for j in range(6)]]:
        result[field] = difference(field)
    result["copper_heat_j"] = sum(result[f"winding_{phase}_heat_j"] for phase in "abc")
    result["energy_residual_j"] = (difference("input_j")-difference("mechanical_work_j")-heat
                                   -difference("magnetic_energy_j")-result["numerical_loss_j"])
    samples = [r for r in rows if start < r["time_s"] <= end and r["command_sample"]]
    result["mean_speed_rad_s"] = sum(r["speed_rad_s"] for r in samples)/len(samples)
    result["peak_winding_current_a"] = rows[-1]["peak_winding_current_a"]
    return result


def normalize(stats, baseline, tracking_limit=2, work_tolerance=.02, regression_limit=.1):
    """Return explicit validity/status. Failed tracking never gets a heat score.
    Work is passive-brake absorption (positive even in reverse), so the
    independent signed-speed tracking check is essential.
    """
    for item in (stats, baseline):
        if not all(math.isfinite(item[k]) and item[k] >= 0 for k in ("heat_j", "load_work_j", "tracking_rms_rad_s")):
            raise ValueError("finite nonnegative heat/work/RMS required")
    if not all(math.isfinite(v) and v >= 0 for v in (tracking_limit, work_tolerance, regression_limit)):
        raise ValueError("finite nonnegative tolerances required")
    result = dict(stats)
    valid_baseline = (baseline["heat_j"] > 1e-12 and baseline["load_work_j"] > 1e-12
                      and baseline["tracking_rms_rad_s"] <= tracking_limit)
    tracking = stats["tracking_rms_rad_s"] <= tracking_limit
    matched_work = (valid_baseline and stats["load_work_j"] > 1e-12
                    and abs(stats["load_work_j"]/baseline["load_work_j"]-1) <= work_tolerance)
    result["score_valid"] = int(valid_baseline and tracking and matched_work)
    ratio = None
    if result["score_valid"]:
        ratio = ((stats["heat_j"]/stats["load_work_j"])
                 /(baseline["heat_j"]/baseline["load_work_j"]))
    result["heat_per_work_ratio"] = ratio
    result["heat_regression_pct"] = None if ratio is None else 100*(ratio-1)
    result["status"] = ("baseline_invalid" if not valid_baseline else
                        "tracking_failed" if not tracking else
                        "work_mismatch" if not matched_work else
                        "heat_regression" if ratio > 1+regression_limit else "pass")
    return result


def read_summary(path):
    import csv
    fields = ["window_start_s", "window_end_s", "heat_j", "load_work_j", "tracking_rms_rad_s"]
    result = {}
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            name = row["case"]
            if not name or name in result:
                raise ValueError("empty or duplicate case in summary")
            result[name] = {field: float(row[field]) for field in fields}
    if not result:
        raise ValueError("empty summary")
    return result


def compare_summaries(current, baseline, names=None, **limits):
    names = list(current) if names is None else names
    if not names or len(set(names)) != len(names):
        raise ValueError("nonempty unique cases required")
    result = []
    for name in names:
        if name not in current or name not in baseline:
            raise ValueError(f"missing case: {name}")
        a, b = current[name], baseline[name]
        for row in (a, b):
            start, end = row["window_start_s"], row["window_end_s"]
            if not (math.isfinite(start) and math.isfinite(end) and 0 <= start < end):
                raise ValueError("invalid task window")
        if any(abs(a[k]-b[k]) > 1e-9 for k in ("window_start_s", "window_end_s")):
            raise ValueError("different task windows")
        scored = normalize(a, b, **limits)
        result.append(dict(case=name, status=scored["status"], heat_per_work_ratio=scored["heat_per_work_ratio"],
                           heat_regression_pct=scored["heat_regression_pct"],
                           tracking_rms_rad_s=a["tracking_rms_rad_s"], load_work_j=a["load_work_j"]))
    return result


def main():
    """Compare a new run against a retained prior baseline; suitable for CI.
    Exit 0 pass, 1 heat/tracking/work regression, 2 invalid input/config.
    No plotting or third-party Python packages needed.
    """
    import argparse
    import csv
    import sys
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument("current")
    parser.add_argument("baseline")
    parser.add_argument("--case", action="append", dest="cases")
    parser.add_argument("--regression-limit-percent", type=float, default=10)
    parser.add_argument("--tracking-limit", type=float, default=2)
    parser.add_argument("--work-tolerance-percent", type=float, default=2)
    args = parser.parse_args()
    try:
        results = compare_summaries(read_summary(args.current), read_summary(args.baseline), args.cases,
            tracking_limit=args.tracking_limit, work_tolerance=args.work_tolerance_percent/100,
            regression_limit=args.regression_limit_percent/100)
    except (OSError, KeyError, TypeError, ValueError, csv.Error) as error:
        print(f"heat check failed: {error}", file=sys.stderr)
        return 2
    writer = csv.DictWriter(sys.stdout, fieldnames=list(results[0]), lineterminator="\n")
    writer.writeheader(); writer.writerows(results)
    return int(any(row["status"] != "pass" for row in results))


if __name__ == "__main__":
    raise SystemExit(main())
