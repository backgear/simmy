import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'examples'))
import axis_phase_probes as phase


class PhaseProbeTests(unittest.TestCase):
    def test_waveform_bounded_and_order_balanced(self):
        values, legs = phase.waveform()
        self.assertEqual(len(values), 600)
        self.assertEqual(values[0], 0)
        self.assertEqual(values[-1], 0)
        self.assertEqual(len(legs), 6)
        self.assertEqual(sum(v != 0 for v in values), 12)
        self.assertLessEqual(max(abs(v) for v in values), 3276)
        self.assertEqual([l['code'] for l in legs], [1638, -1638, 3276, -1638, 1638, 3276])

    def test_large_profile_spacing_and_counterbalance(self):
        for batch in (1, 2):
            values, legs = phase.waveform('large', batch)
            initial = 1 if batch == 1 else -1
            self.assertEqual(len(values), 600)
            self.assertEqual([l['code'] for l in legs], [initial*(-1)**i*3276 for i in range(6)])
            self.assertEqual([l['start_sample'] for l in legs], [20, 130, 240, 350, 460, 570])
            self.assertTrue(all(l['end_sample']-l['start_sample'] == 4 for l in legs))
            self.assertEqual(values[-1], 0)

    def test_analysis_timebase_phase_and_endpoint(self):
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);case=root/'batch-01';case.mkdir()
            values,legs=phase.waveform()
            (root/'health.json').write_text(json.dumps(dict(hello=dict(tick_hz=1000000))))
            (case/'report.json').write_text(json.dumps(dict(passed=True,state='COMPLETE')))
            (case/'plan.json').write_text(json.dumps(dict(commands=values,manifest=dict(command_period_ticks=5000))))
            desc=dict(stream_id=0,sample_count=8201,period_ticks=500,first_sample_tick=0)
            (case/'get_run.json').write_text(json.dumps(dict(command_start_tick=100000,streams=[desc])))
            v=[]
            for i in range(8201):
                count=10
                for leg in legs:
                    if i*500 >= 100000+leg['end_sample']*5000:
                        count += (500 if leg['role']=='index' else 4 if leg['code']>0 else -4)
                v.append(count)
            (case/'stream-0.i64le').write_bytes(struct.pack('<'+'q'*len(v),*v))
            metrics=phase.analyze(case,1)
            self.assertEqual(len(metrics),6)
            self.assertTrue(all(m['stationary_windows'] for m in metrics))
            self.assertEqual([m['displacement_counts'] for m in metrics],[4,-4,500,-4,4,500])
            self.assertEqual([m['site'] for m in metrics],[0,0,0,1,1,1])
            self.assertEqual(metrics[0]['command_start_tick'],250000)
            self.assertAlmostEqual(metrics[0]['initial_phase_deg_arbitrary_zero'],10*360/4096)


if __name__ == '__main__':
    unittest.main()
