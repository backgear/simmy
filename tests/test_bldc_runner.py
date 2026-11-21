import csv
import io
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "examples"))
import run_bldc_sweep as runner

FAKE = '''#!/usr/bin/env python3
import os,sys,time
if "--list" in sys.argv:
 print("healthy\\nfault"); sys.exit(0)
name=sys.argv[sys.argv.index("--case")+1]
mode=os.environ.get("BLDC_MOCK_MODE", "ok")
if name=="healthy": time.sleep(.02)
if name=="fault" and mode=="fail": sys.exit(7)
header="case,other" if name=="fault" and mode=="header" else "case,value"
value="nan" if mode=="nan" else "1.25"
print(header)
if mode!="empty": print(("wrong" if mode=="wrong" else name)+","+value)
'''


class BldcRunnerTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.binary = Path(self.directory.name)/"mock"
        self.binary.write_text(FAKE)
        self.binary.chmod(0o755)

    def test_default_overcommit(self):
        self.assertEqual(runner.default_jobs(), 256)

    def test_order_and_byte_identical_serial(self):
        serial = runner.run_sweep(self.binary, ["healthy", "fault"], 4, 1)
        parallel = runner.run_sweep(self.binary, ["healthy", "fault"], 4, 256)
        self.assertEqual(serial, parallel)
        self.assertEqual(list(csv.reader(io.StringIO(parallel))),
                         [["case", "value"], ["healthy", "1.25"], ["fault", "1.25"]])

    def test_bad_outputs_fail(self):
        for mode in ["fail", "header", "nan", "wrong", "empty"]:
            with self.subTest(mode=mode), patch.dict(os.environ, {"BLDC_MOCK_MODE": mode}):
                with self.assertRaises(RuntimeError):
                    runner.run_sweep(self.binary, ["healthy", "fault"], 4, 256)

    def test_failed_cli_preserves_previous_output(self):
        output = Path(self.directory.name)/"previous.csv"
        output.write_text("keep me\n")
        with patch.dict(os.environ, {"BLDC_MOCK_MODE": "fail"}):
            result = subprocess.run([sys.executable, str(ROOT/"examples/run_bldc_sweep.py"),
                                     "--binary", str(self.binary), "--output", str(output)], capture_output=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertEqual(output.read_text(), "keep me\n")

    def test_subset_atomic_success(self):
        output = Path(self.directory.name)/"new"/"result.csv"
        result = subprocess.run([sys.executable, str(ROOT/"examples/run_bldc_sweep.py"),
                                 "--binary", str(self.binary), "--case", "fault", "--output", str(output)], capture_output=True)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(output.read_text(), "case,value\nfault,1.25\n")
        self.assertEqual(list(output.parent.iterdir()), [output])

    def test_invalid_configuration(self):
        for names, jobs, steps in [([], 1, 1), (["healthy"], 0, 1), (["healthy"], 1, 0)]:
            with self.assertRaises(ValueError):
                runner.run_sweep(self.binary, names, steps, jobs)
        for arguments in [["--jobs", "0"], ["--case", "unknown"], ["--case", "fault", "--case", "fault"]]:
            result = subprocess.run([sys.executable, str(ROOT/"examples/run_bldc_sweep.py"),
                                     "--binary", str(self.binary), *arguments], capture_output=True)
            self.assertNotEqual(result.returncode, 0)


if __name__ == "__main__":
    unittest.main()
