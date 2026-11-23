import shutil
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_model_fit import Model
from axis_model_fresh_validation import waveform


@unittest.skipUnless(shutil.which('cc'),'C compiler required for offline kernel')
class CommandModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();cls.model=Model(cls.tmp.name)

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_stiction_holds_constant_bias(self):
        x,v=self.model.simulate(np.zeros((1,1000)),[1000],.0005,[1000,10,20,0,5,0,.0001,0])
        np.testing.assert_array_equal(x,0);np.testing.assert_array_equal(v,0)

    def test_pulse_stops_without_repeated_state_reset(self):
        u=np.zeros((2,2001));u[0,:40]=.1;u[1,:40]=-.1
        x,v=self.model.simulate(u,[2001,2001],.0005,[1000,0,10,0,5,0,.0001,0])
        self.assertAlmostEqual(x[0,-1],.18,delta=.004)
        self.assertGreater(np.max(v[0]),1.7)
        self.assertEqual(v[0,-1],0)
        np.testing.assert_allclose(x[0],-x[1],atol=1e-12)
        np.testing.assert_allclose(v[0],-v[1],atol=1e-12)

    def test_deadband_suppresses_small_input(self):
        x,v=self.model.simulate(np.ones((1,100))*.01,[100],.0005,[1000,0,10,0,5,0,.001,.02])
        np.testing.assert_array_equal(x,0)

    def test_registered_fresh_waveform_is_bounded(self):
        u=waveform();self.assertEqual(len(u),600)
        self.assertEqual((u[0],u[-1]),(0,0))
        for i,start in enumerate((20,160,300,440)):
            s=(-1)**i
            self.assertEqual(u[start:start+3],[s*4915]*3)
            self.assertEqual(u[start+11],-s*1638)
            self.assertEqual(u[start+3:start+11],[0]*8)


if __name__=='__main__':unittest.main()
