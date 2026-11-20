"""Two-body fitting regressions; optional NumPy + SciPy dependencies."""
import copy
import importlib.util
from pathlib import Path
import unittest

import numpy as np


def load(name):
    path = Path(__file__).resolve().parents[1] / 'examples' / f'{name}.py'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


axis = load('identify_axis')
compliance = load('axis_compliance')


class ComplianceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = axis.make_plan(hz=500, high=30)
        cls.rows = compliance.demo_compliance(cls.plan)
        cls.report, cls.predictions = compliance.fit_compliance(cls.rows, .02, .005)

    def test_recovers_parameters_and_trajectory(self):
        p = self.report['parameters']
        for name, expected in [('motor_inertia', 1.2), ('load_inertia', .8),
                               ('stiffness', 180), ('damping', 1.5)]:
            self.assertAlmostEqual(p[name], expected, delta=.06*expected)
        self.assertAlmostEqual(p['backlash_half_gap'], .002, delta=.00005)
        self.assertGreater(self.report['gap_sse_improvement_fraction'], .9)
        self.assertLess(self.report['validation']['load_position_rmse'], .0002)
        self.assertEqual(len(self.predictions), sum(r['segment'] == 'validation' for r in self.rows))

    def test_validation_not_used_for_fit(self):
        rows = copy.deepcopy(self.rows)
        for row in rows:
            if row['segment'] == 'validation':
                row['effort'] += .1
        report, _ = compliance.fit_compliance(rows, .02, .005)
        self.assertEqual(report['parameters'], self.report['parameters'])
        self.assertGreater(report['validation']['load_position_rmse'], .001)

    def test_zero_gap_and_small_sensor_noise(self):
        rows = compliance.demo_compliance(self.plan, gap=0)
        rng = np.random.default_rng(7)
        for row in rows:
            row['motor_position'] += rng.normal(0, 1e-8)
            row['load_position'] += rng.normal(0, 1e-8)
        report, _ = compliance.fit_compliance(rows, .02, .005)
        self.assertLess(report['parameters']['backlash_half_gap'], .00005)
        self.assertAlmostEqual(report['parameters']['stiffness'], 180, delta=10)

    def test_missing_load_and_one_sided_data_rejected(self):
        rows = copy.deepcopy(self.rows)
        del rows[0]['load_position']
        with self.assertRaisesRegex(ValueError, 'load_position'):
            compliance.fit_compliance(rows, .02, .005)
        for row in rows:
            row['load_position'] = row['motor_position']-.001
        with self.assertRaisesRegex(ValueError, 'both signs'):
            compliance.fit_compliance(rows, .02, .005)

    def test_deadzone_convention(self):
        np.testing.assert_allclose(compliance.deadzone(np.array([-.003, -.001, .001, .003]), .002),
                                   [-.001, 0, 0, .001])


if __name__ == '__main__':
    unittest.main()
