"""Optional stdlib-only tests: make test-tools (C-only make test unchanged)."""
from contextlib import redirect_stderr
import importlib.util
import io
from pathlib import Path
import tempfile
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "sweep_runner", Path(__file__).resolve().parents[1] / "examples/run_stepper_sweep.py")
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


class SweepRunnerTests(unittest.TestCase):
    def test_default_allows_overcommit(self):
        self.assertEqual(runner.default_jobs(), 256)

    def test_workers_are_capped_by_trials_not_machine_size(self):
        with patch.object(runner, "ThreadPoolExecutor", wraps=runner.ThreadPoolExecutor) as pool, \
             patch.object(runner, "point", return_value=(["value"], [["1"]])):
            runner.run_sweep(Path("unused"), [.1, .2], 10000, 512)
        pool.assert_called_once_with(max_workers=2)

    def test_parallel_execution_keeps_speed_order(self):
        second_finished = threading.Event()
        completed = []

        def point(binary, speed, hz):
            if speed == .01:
                self.assertTrue(second_finished.wait(timeout=5))
            completed.append(speed)
            if speed == .02:
                second_finished.set()
            return ["speed", "hz"], [[str(speed), str(hz)]]

        with patch.object(runner, "point", side_effect=point):
            result = runner.run_sweep(Path("unused"), [.01, .02, .03], 10000, 2)
        self.assertEqual(result, "speed,hz\n0.01,10000\n0.02,10000\n0.03,10000\n")
        self.assertLess(completed.index(.02), completed.index(.01))

    def test_failed_subprocess_propagates_stderr(self):
        result = SimpleNamespace(returncode=2, stderr="bad parameter", stdout="")
        with patch.object(runner.subprocess, "run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "0.2 rad/s.*bad parameter"):
                runner.point(Path("binary"), .2, 10000)

    def test_malformed_csv_is_rejected(self):
        result = SimpleNamespace(returncode=0, stderr="", stdout="a,b\n1\n")
        with patch.object(runner.subprocess, "run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "invalid CSV"):
                runner.point(Path("binary"), .2, 10000)

    def test_inconsistent_headers_are_rejected(self):
        def point(binary, speed, hz):
            return [str(speed)], [["value"]]
        with patch.object(runner, "point", side_effect=point):
            with self.assertRaisesRegex(RuntimeError, "inconsistent CSV headers"):
                runner.run_sweep(Path("unused"), [.1, .2], 10000, 2)

    def test_atomic_output_replaces_file_without_leftovers(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "nested" / "data.csv"
            runner.write_atomic(path, "old")
            runner.write_atomic(path, "new")
            self.assertEqual(path.read_text(), "new")
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_failure_leaves_existing_output_untouched(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "data.csv"
            path.write_text("previous complete result")
            args = ["runner", "compensation", "--output", str(path)]
            with patch.object(runner.sys, "argv", args), \
                 patch.object(runner, "run_sweep", side_effect=RuntimeError("worker failed")), \
                 redirect_stderr(io.StringIO()):
                self.assertEqual(runner.main(), 1)
            self.assertEqual(path.read_text(), "previous complete result")

    def test_invalid_arguments(self):
        for options in [["--jobs", "0"], ["--points", "1"], ["--min-speed", "nan"]]:
            with patch.object(runner.sys, "argv", ["runner", "compensation", *options]), \
                 redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    runner.main()
                self.assertEqual(error.exception.code, 2)


if __name__ == "__main__":
    unittest.main()
