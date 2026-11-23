import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'examples'))
import axis_pulse_report as pulse
import axis_pulse_trials as trials


class PulseReportTests(unittest.TestCase):
    def fixture(self, root, direction=1):
        case = root / 'case'
        case.mkdir()
        (root / 'health.json').write_text(json.dumps({'hello': {'tick_hz': 1000000}}))
        plan = {'run_id': 'test', 'commands': [0]*20 + [3276*direction]*20 + [0]*60,
                'manifest': {'command_period_ticks': 1000, 'analog_output_full_scale': 32767}}
        (case / 'plan.json').write_text(json.dumps(plan))
        run = {'command_start_tick': 100000, 'streams': [dict(stream_id=0, sample_count=1201,
                channels=1, period_ticks=1000, first_sample_tick=0)]}
        (case / 'get_run.json').write_text(json.dumps(run))
        (case / 'report.json').write_text(json.dumps(dict(passed=True, state='COMPLETE')))
        values = [10 + direction * min(200, max(0, i-120)*2) for i in range(1201)]
        (case / 'stream-0.i64le').write_bytes(struct.pack('<'+'q'*len(values), *values))
        return case

    def test_positive_and_negative_signs_and_timebase(self):
        for direction in (-1, 1):
            with self.subTest(direction=direction), tempfile.TemporaryDirectory() as tmp:
                case = self.fixture(Path(tmp), direction)
                summary, rows = pulse.summarize(case)
                self.assertEqual(summary['displacement_counts'], direction*200)
                self.assertEqual(summary['direction'], direction)
                self.assertTrue(summary['stationary_tail'])
                self.assertAlmostEqual(summary['pulse_duration_s'], .02)
                self.assertAlmostEqual(rows[0]['time_s'], -.1)
                self.assertEqual(rows[120]['command_code'], direction*3276)
                self.assertEqual(rows[140]['command_code'], 0)
                self.assertEqual(rows[-1]['segment'], 'post_roll')
                self.assertEqual(rows[-1]['velocity_deg_s'], 0)
                self.assertTrue((case / 'motion.csv').exists())

    def test_reject_faulted_recording(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.fixture(Path(tmp))
            (case / 'report.json').write_text(json.dumps(dict(passed=True, state='FAULT')))
            with self.assertRaises(ValueError):
                pulse.summarize(case)

    def test_reject_truncated_raw(self):
        with tempfile.TemporaryDirectory() as tmp:
            case = self.fixture(Path(tmp))
            path = case / 'stream-0.i64le'
            path.write_bytes(path.read_bytes()[:-1])
            with self.assertRaises(ValueError):
                pulse.summarize(case)


class TrialPlanTests(unittest.TestCase):
    def test_alternating_bounded_plan(self):
        plan = trials.trial_plan(10, 20, 3)
        self.assertEqual([p['direction'] for p in plan], [1, -1, 1, -1, 1, -1])
        for p in plan:
            self.assertEqual(len(p['commands']), 100)
            self.assertEqual(p['commands'][:20], [0]*20)
            self.assertEqual(p['commands'][20:40], [3276*p['direction']]*20)
            self.assertEqual(p['commands'][40:], [0]*60)

    def test_reject_outside_authorized_envelope(self):
        for args in ((11, 20, 1), (10, 21, 1), (10, 20, 4), (0, 20, 1),
                     (float('nan'), 20, 1), (10, 0, 1)):
            with self.subTest(args=args), self.assertRaises(ValueError):
                trials.trial_plan(*args)


if __name__ == '__main__':
    unittest.main()
