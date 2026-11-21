import sys
from pathlib import Path
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/"examples"))
from bldc_metrics import normalize, compare_summaries, window_stats
import csv
import subprocess
import tempfile


class HeatMetricTests(unittest.TestCase):
    def setUp(self):
        self.baseline = dict(heat_j=2., load_work_j=10., tracking_rms_rad_s=.5)

    def test_baseline_and_regression(self):
        self.assertEqual(normalize(self.baseline, self.baseline)["heat_per_work_ratio"], 1)
        result = normalize(dict(self.baseline, heat_j=3.), self.baseline)
        self.assertEqual(result["status"], "heat_regression")
        self.assertAlmostEqual(result["heat_regression_pct"], 50)

    def test_cool_failed_task_is_not_an_improvement(self):
        for bad in [dict(self.baseline, heat_j=.1, load_work_j=0),
                    dict(self.baseline, heat_j=.1, tracking_rms_rad_s=120)]:
            result = normalize(bad, self.baseline)
            self.assertEqual(result["score_valid"], 0)
            self.assertIsNone(result["heat_per_work_ratio"])

    def test_reverse_absorbs_work_but_fails_tracking(self):
        result = normalize(dict(self.baseline, tracking_rms_rad_s=120), self.baseline)
        self.assertEqual(result["status"], "tracking_failed")

    def test_invalid_zero_or_failed_baseline(self):
        for b in [dict(self.baseline, heat_j=0), dict(self.baseline, load_work_j=0),
                  dict(self.baseline, tracking_rms_rad_s=20)]:
            self.assertEqual(normalize(self.baseline, b)["status"], "baseline_invalid")

    def test_historical_baseline_and_window_guard(self):
        base = dict(self.baseline, window_start_s=.5, window_end_s=1.5)
        result = compare_summaries({"healthy": dict(base, heat_j=3)}, {"healthy": base})
        self.assertEqual(result[0]["status"], "heat_regression")
        for current in [{"healthy": dict(base, window_end_s=2)}, {}, {"other": base}]:
            with self.assertRaises(ValueError):
                compare_summaries(current, {"healthy": base})

    def test_cli_exit_status(self):
        base = dict(case="healthy", window_start_s=.5, window_end_s=1.5, **self.baseline)
        script = Path(__file__).resolve().parents[1]/"examples/bldc_metrics.py"
        with tempfile.TemporaryDirectory() as directory:
            baseline = Path(directory)/"base.csv"
            current = Path(directory)/"current.csv"
            def write(path, data):
                with path.open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(data)); writer.writeheader(); writer.writerow(data)
            write(baseline, base)
            for heat, expected in [(2, 0), (3, 1)]:
                write(current, dict(base, heat_j=heat))
                result = subprocess.run([sys.executable, str(script), str(current), str(baseline)], capture_output=True)
                self.assertEqual(result.returncode, expected, result.stderr)
            write(current, dict(base, window_end_s=2))
            result = subprocess.run([sys.executable, str(script), str(current), str(baseline)], capture_output=True)
            self.assertEqual(result.returncode, 2)

    def test_mixed_rate_rows_do_not_bias_integrals(self):
        def row(time, command=1):
            r = dict(time_s=time, command_sample=command, heat_j=2*time, load_work_j=10*time,
                     error_square_integral=.25*time, numerical_loss_j=.1*time, diode_heat_j=0,
                     switch_heat_j=0, input_j=12.1*time, mechanical_work_j=10*time,
                     magnetic_energy_j=0, speed_rad_s=60, peak_winding_current_a=2)
            for phase in "abc": r[f"winding_{phase}_heat_j"] = 2/3*time
            for j in range(6): r[f"fet{j}_heat_j"] = 0
            return r
        sparse = [row(.5), row(1), row(1.5)]
        dense = sparse[:1]+[row(.7+i*.001, 0) for i in range(100)]+sparse[1:]
        self.assertEqual(window_stats(sparse), window_stats(dense))
        self.assertAlmostEqual(window_stats(sparse)["energy_residual_j"], 0)
        with self.assertRaises(ValueError): window_stats(sparse, .4, 1.5)

    def test_nonfinite_inputs_rejected(self):
        for value in [float("nan"), float("inf"), -1]:
            with self.assertRaises(ValueError):
                normalize(dict(self.baseline, heat_j=value), self.baseline)
        result = normalize(dict(self.baseline, load_work_j=0), self.baseline, work_tolerance=2)
        self.assertEqual(result["score_valid"], 0)

    def test_work_normalization_and_tolerance(self):
        result = normalize(dict(self.baseline, heat_j=2.02, load_work_j=10.1), self.baseline)
        self.assertAlmostEqual(result["heat_per_work_ratio"], 1)
        self.assertEqual(normalize(dict(self.baseline, load_work_j=11), self.baseline)["status"], "work_mismatch")


if __name__ == "__main__":
    unittest.main()
