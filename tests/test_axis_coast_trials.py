import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
import axis_recorder as recorder
import axis_coast_speed_trials as coast
from axis_seed_validation import prediction


class CoastTrialTests(unittest.TestCase):
    def test_bounded_launch_and_coast_waveforms(self):
        for batch in (1,2):
            for percent in (0,1):
                v=coast.waveform(batch,percent)
                self.assertEqual(len(v),600)
                self.assertEqual((v[0],v[-1]),(0,0))
                self.assertEqual(sum(abs(x)==6553 for x in v),8)
                self.assertEqual(sum(x==327 for x in v),400 if percent else 0)
                for i,start in enumerate((20,160,300,440)):
                    sign=(1 if batch==1 else -1)*(-1)**i
                    self.assertEqual(v[start:start+2],[sign*6553]*2)
                    self.assertEqual(v[start+102:start+110],[0]*8)

    def test_default_ceiling_cannot_silently_escalate(self):
        with tempfile.TemporaryDirectory() as tmp:
            t=SimpleNamespace(directory=Path(tmp))
            h=dict(tick_hz=1000000,max_command_samples=600,analog_min_period_ticks=250)
            with self.assertRaisesRegex(ValueError,'3276'):
                recorder.record(t,h,'reject',[0],False,500,.1,command_values=[0,6553,0])

    def test_explicit_ceiling_is_preserved_in_plan(self):
        with tempfile.TemporaryDirectory() as tmp:
            def request(*args,**kwargs):raise RuntimeError('offline transport; do not move')
            t=SimpleNamespace(directory=Path(tmp),request=request)
            h=dict(tick_hz=1000000,max_command_samples=600,analog_min_period_ticks=250)
            r=recorder.record(t,h,'explicit',[0],False,500,6,
                command_values=coast.waveform(),command_ceiling_code=6553)
            self.assertFalse(r['passed'])
            plan=json.loads((Path(tmp)/'explicit'/'plan.json').read_text())
            self.assertEqual(plan['authorized_command_ceiling_code'],6553)
            self.assertEqual(plan['manifest']['command_sample_count'],600)
            self.assertEqual(plan['manifest']['command_period_ticks'],10000)

    def test_constant_drag_exact_stopping_position(self):
        t=np.array([-.1,0,.02,.10,.20,1.])
        p=prediction(t,0,.02,.1,1000,10,0)
        np.testing.assert_allclose(p,[0,0,.018,.13,.18,.18],atol=1e-10)
        np.testing.assert_allclose(prediction(t,0,.02,-.1,1000,10,0),-p)
        np.testing.assert_allclose(prediction(t,0,.02,.01,100,10,0),0)


if __name__=='__main__':unittest.main()
