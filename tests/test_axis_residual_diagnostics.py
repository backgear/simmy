import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples'))
from axis_residual_diagnostics import (verify_effort_capture, static_bounds,
                                       hold_windows, maneuver_rows, position_velocity,
                                       quadratic_kinematics, verify_phase_chain,
                                       summarize_driven)


class ResidualDiagnosticsTests(unittest.TestCase):
    def fixture(self, root):
        case = root / 'capture'
        case.mkdir()
        raw = np.array([0, 1, -2, 0], dtype='<i8').tobytes()
        (case / 'stream-0.i64le').write_bytes(raw)
        digest = hashlib.sha256(raw).hexdigest()
        commands = [0, -491, 0]
        command_hash = hashlib.sha256(np.array(commands, dtype='<i8').tobytes()).hexdigest()
        plan = dict(run_id='test', commands=commands, manifest=dict(
            output_mode='analog_effort', analog_enabled=False, analog_output_full_scale=32767,
            encoder_sample_period_ticks=500, command_sample_count=3))
        stream = dict(stream_id=0, sha256=digest, sample_count=4,
                      period_ticks=500, first_sample_tick=100)
        run = dict(run_id='test', command_hash=command_hash, capture_complete=True,
                   fault_reason=None, capture_start_tick=100, streams=[stream])
        report = dict(passed=True, state='COMPLETE', errors=[], streams={'0': dict(
            sha256=digest, sample_count=4, first_tick=100)})
        for name, data in [('plan.json', plan), ('get_run.json', run), ('report.json', report)]:
            (case / name).write_text(json.dumps(data))
        return case

    def test_integrity_and_i64_command_hash(self):
        with tempfile.TemporaryDirectory() as d:
            case = self.fixture(Path(d))
            self.assertIn('stream-0.i64le', verify_effort_capture(case))
            plan = json.loads((case / 'plan.json').read_text())
            run = json.loads((case / 'get_run.json').read_text())
            run['command_hash'] = hashlib.sha256(np.array(plan['commands'], dtype='<i4').tobytes()).hexdigest()
            (case / 'get_run.json').write_text(json.dumps(run))
            with self.assertRaisesRegex(ValueError, 'command hash'): verify_effort_capture(case)

    def test_rejects_corrupted_encoder(self):
        with tempfile.TemporaryDirectory() as d:
            case = self.fixture(Path(d))
            (case / 'stream-0.i64le').write_bytes(bytes(32))
            with self.assertRaisesRegex(ValueError, 'encoder hash'): verify_effort_capture(case)

    def test_rejects_fault_despite_passed_report(self):
        with tempfile.TemporaryDirectory() as d:
            case = self.fixture(Path(d))
            run = json.loads((case / 'get_run.json').read_text())
            run['fault_reason'] = 'watchdog'
            (case / 'get_run.json').write_text(json.dumps(run))
            with self.assertRaisesRegex(ValueError, 'faulted'): verify_effort_capture(case)

    def test_static_bound_units_and_direction(self):
        p = dict(command_deadband_fraction=.01,
                 positive_constant_loss_rad_s2=20, negative_constant_loss_rad_s2=60,
                 positive_gain_rad_s2_per_command=1000, negative_gain_rad_s2_per_command=2000)
        lo, hi = static_bounds(p)
        self.assertAlmostEqual(lo, -.04 * 32767)
        self.assertAlmostEqual(hi, .03 * 32767)

    def test_hold_uses_every_encoder_sample_and_every_command(self):
        count = np.zeros(799, dtype=np.int64)
        telemetry = np.column_stack([np.full(400, 830), np.ones(400)])
        rows = hold_windows(count, telemetry, 0, 400, (-190, 626), 'test', 'development')
        self.assertEqual(len(rows), 2)
        self.assertTrue(all(r['every_sample_beyond_one_static_bound'] for r in rows))
        count[599] = 1  # Last half-millisecond sample in the first window.
        telemetry[250, 0] = 0  # High mean effort is not sufficient evidence.
        rows = hold_windows(count, telemetry, 0, 400, (-190, 626), 'test', 'development')
        self.assertEqual(rows[0]['encoder_span_counts'], 1)
        self.assertFalse(rows[0]['every_sample_beyond_one_static_bound'])
        self.assertTrue(rows[1]['every_sample_beyond_one_static_bound'])

    def test_error_change_does_not_reset_prediction_state(self):
        t = np.arange(-.1, .501, .0005)
        measured = .4 + t + .1
        predicted = 100 + 2 * (t + .1)
        original = predicted.copy()
        moves = [dict(start=10, width=5, code=491, group='weak_hold')]
        rows = maneuver_rows(t, measured, predicted, np.full(len(t), 2), moves, {})
        row = rows[0]
        self.assertGreater(row['error_at_start_deg'], 5000)
        self.assertAlmostEqual(row['error_change_deg'], row['predicted_increment_deg'] - row['measured_increment_deg'])
        self.assertEqual(row['predicted_start_velocity_rad_s'], 2)
        np.testing.assert_array_equal(predicted, original)

    def test_boot_id_alone_does_not_establish_phase_continuity(self):
        first = dict(boot_id=7, first_tick=100, last_tick=200, first_count=10, last_count=11)
        second = dict(boot_id=7, first_tick=201, last_tick=300, first_count=11, last_count=12)
        verify_phase_chain([first, second])
        with self.assertRaisesRegex(ValueError, 'clocks'):
            verify_phase_chain([first, dict(second, first_tick=150)])
        with self.assertRaisesRegex(ValueError, 'encoder continuity'):
            verify_phase_chain([first, dict(second, first_count=0)])

    def test_driven_summary_weights_moves_not_window_counts(self):
        base = dict(suite='test', source='capture', code=-491)
        rows = [dict(base, move=1, conditional_force_ratio=4) for _ in range(9)]
        rows.append(dict(base, move=2, conditional_force_ratio=0))
        summary = summarize_driven(rows, ['code'])[0]
        self.assertEqual(summary['moves'], 2)
        self.assertEqual(summary['windows'], 10)
        self.assertEqual(summary['median_per_move_force_ratio'], 2)

    def test_kinematic_derivatives_of_quadratic(self):
        t = np.arange(-20, 21) * .0005
        v, a, error = quadratic_kinematics(20 + 3*t + .5*80*t*t, .0005)
        self.assertAlmostEqual(v, 3)
        self.assertAlmostEqual(a, 80)
        self.assertLess(error, 1e-9)

    def test_velocity_estimator_preserves_constant_speed(self):
        t = np.arange(100) * .0005
        np.testing.assert_allclose(position_velocity(3 * t + 20, .0005), 3, atol=1e-10)


if __name__ == '__main__': unittest.main()
