"""Offline identification smoke/regression tests (NumPy required)."""
import importlib.util
from pathlib import Path
import tempfile
import unittest
import subprocess
import sys

spec = importlib.util.spec_from_file_location(
    'identify_axis', Path(__file__).resolve().parents[1] / 'examples/identify_axis.py')
axis = importlib.util.module_from_spec(spec)
spec.loader.exec_module(axis)


class IdentificationTests(unittest.TestCase):
    def test_plan_bounds_and_holdout(self):
        rows = axis.make_plan(hz=500)
        self.assertEqual(set(r['segment'] for r in rows),
                         {'friction', 'acceleration', 'dynamic', 'validation'})
        self.assertTrue(all(0 <= r['target'] <= .01 for r in rows))
        self.assertTrue(all(abs(r['excitation']) <= .1 for r in rows))
        self.assertTrue(all(b['time'] > a['time'] for a, b in zip(rows, rows[1:])))
        self.assertTrue(all(r['excitation'] == 0 for r in rows if r['segment'] != 'dynamic'))

    def test_synthetic_fit_and_holdout(self):
        rows = axis.demo(axis.make_plan())
        report = axis.fit(rows)
        for key, value in [('inertia', 2), ('viscous', .7), ('coulomb', .015), ('bias', .01)]:
            self.assertAlmostEqual(report['parameters'][key], value, delta=.05*abs(value))
        self.assertLess(report['validation']['relative_error'], .05)
        # Changing held-out effort must not change trained parameters.
        for row in rows:
            if row['segment'] == 'validation':
                row['effort'] += 1
        changed = axis.fit(rows)
        self.assertEqual(changed['parameters'], report['parameters'])
        self.assertGreater(changed['validation']['relative_error'], .5)

    def test_csv_validation(self):
        rows = axis.demo(axis.make_plan(hz=200, high=10))[:30]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'data.csv'
            axis.write_csv(path, axis.MEASURE_FIELDS, rows)
            self.assertEqual(len(axis.read_measurements(path)), 30)
            rows[1]['time'] = rows[0]['time']
            axis.write_csv(path, axis.MEASURE_FIELDS, rows)
            with self.assertRaisesRegex(ValueError, 'strictly increase'):
                axis.read_measurements(path)

    def test_capture_adapter_and_overwrite_guard(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            plan, fixture, output = (root / name for name in ('plan.csv', 'fixture.csv', 'out.csv'))
            rows = axis.make_plan(hz=200, high=10)
            axis.write_csv(plan, axis.PLAN_FIELDS, rows)
            axis.write_csv(fixture, axis.MEASURE_FIELDS, axis.demo(rows)[:30])
            adapter = root / 'adapter'
            adapter.write_text(
                f'#!{sys.executable}\nimport argparse, shutil\n'
                'p=argparse.ArgumentParser()\np.add_argument("--plan")\n'
                'p.add_argument("--output")\na=p.parse_args()\n'
                f'shutil.copyfile({str(fixture)!r}, a.output)\n')
            adapter.chmod(0o755)
            command = [sys.executable, str(Path(axis.__file__)), 'capture',
                       '--plan', str(plan), '--output', str(output), '--adapter', str(adapter)]
            subprocess.run(command, check=True, capture_output=True, text=True)
            self.assertEqual(len(axis.read_measurements(output)), 30)
            result = subprocess.run(command, capture_output=True, text=True)
            self.assertNotEqual(result.returncode, 0)
            self.assertIn('refusing to overwrite', result.stderr)

    def test_unexcited_data_rejected(self):
        rows = axis.demo(axis.make_plan(hz=200, high=10))
        for row in rows:
            row['motor_position'] = 0
        with self.assertRaisesRegex(ValueError, 'training.*validation'):
            axis.fit(rows)


if __name__ == '__main__':
    unittest.main()
