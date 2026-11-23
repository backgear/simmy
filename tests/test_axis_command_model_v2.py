import shutil
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_model_fit import Model as V1
from axis_model_fit_v2 import Model as V2
from axis_model_validate_v2 import plan


@unittest.skipUnless(shutil.which('cc'),'offline kernel needs compiler')
class ModelV2Tests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.v1=V1(cls.tmp.name);cls.v2=V2(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_equivalence_to_biased_v1_when_viscous_rates_equal(self):
        u=np.zeros((1,6001));u[0,200:240]=.1;u[0,2000:2040]=-.1;u[0,4000:4040]=.1;u[0,4140:4160]=-.05
        x1,v1=self.v1.simulate(u,[6001],.0005,[12000,-100,170,0,5,.8,.001,.002])
        x2,v2=self.v2.simulate(u,[6001],.0005,[12000,12000,270,70,.8,.8,.001,.002])
        np.testing.assert_allclose(x1,x2,atol=1e-10)
        np.testing.assert_allclose(v1,v2,atol=1e-10)

    def test_zero_command_stays_still(self):
        x,v=self.v2.simulate(np.zeros((1,100)),[100],.0005,[12000,12000,250,60,1,2,.001,.002])
        np.testing.assert_array_equal(x,0);np.testing.assert_array_equal(v,0)


class ValidationPlanTests(unittest.TestCase):
    def test_designed_pattern_has_longer_moves_and_bounds(self):
        values,moves=plan()
        self.assertEqual(len(values),600);self.assertEqual(len(moves),6)
        self.assertEqual([m['width'] for m in moves],[1,1,2,2,3,3])
        self.assertLessEqual(max(abs(v) for v in values),5898)
        self.assertEqual((values[0],values[-1]),(0,0))

    def test_random_plans_reproduce_and_stay_in_envelope(self):
        for seed in range(30):
            with self.subTest(seed=seed):
                values,moves=plan(seed)
                self.assertEqual((values,moves),plan(seed))
                self.assertEqual(len(values),600)
                self.assertLessEqual(max(abs(v) for v in values),5898)
                self.assertEqual((values[0],values[-1]),(0,0))
                for i in range(0,6,2):self.assertLess(moves[i]['code']*moves[i+1]['code'],0)
                for i,m in enumerate(moves):
                    area=sum(abs(v) for v in values[m['start']:m['start']+100])*.02/32767
                    self.assertLessEqual(area,.0072+1e-12)
                    if 'brake_code' in m:self.assertLess(m['brake_code']*m['code'],0)
        self.assertNotEqual(plan(23210),plan(23211))


if __name__=='__main__':unittest.main()
