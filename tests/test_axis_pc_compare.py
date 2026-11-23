import sys
from pathlib import Path
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_pc_compare import step_metrics

class StepMetricsTests(unittest.TestCase):
    def fixture(self,sign=1):
        reference=np.r_[np.zeros(5),np.ones(100)*100]*sign
        position=reference.copy();position[5:15]=np.array([0,25,50,75,100,120,110,90,98,99])*sign
        return position,reference
    def test_direct_observables_and_envelope(self):
        p,r=self.fixture();v=step_metrics(p,r,len(r))[0]
        self.assertEqual(v['overshoot_percent'],20)
        self.assertEqual(v['rise_10_90_ms'],3)
        self.assertEqual(v['settling_2_percent_ms'],8)
        self.assertEqual(v['envelope_1e_decay_ms'],3)
        self.assertEqual(v['overshoot_to_undershoot_ms'],2)
        self.assertEqual(v['opposite_extremum_ratio'],.5)
        self.assertIsNone(v['ringing_period_ms'])
    def test_direction_symmetry(self):
        a=step_metrics(*self.fixture(),105)[0];b=step_metrics(*self.fixture(-1),105)[0]
        for key in ('overshoot_percent','rise_10_90_ms','settling_2_percent_ms','envelope_1e_decay_ms','overshoot_to_undershoot_ms'):
            self.assertEqual(a[key],b[key])
    def test_never_settled_is_not_fake_time(self):
        _,r=self.fixture();v=step_metrics(np.zeros(len(r)),r,len(r))[0]
        self.assertIsNone(v['settling_2_percent_ms']);self.assertIsNone(v['rise_10_90_ms'])
        self.assertIsNone(v['envelope_1e_decay_ms'])

if __name__=='__main__':unittest.main()
