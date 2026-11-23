import shutil
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'examples'))
from axis_experimental_fit import Experimental,derivative
from axis_model_fit_v2 import Model as V2


@unittest.skipUnless(shutil.which('cc'),'offline kernels need compiler')
class ExperimentalModelsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory()
        cls.base=Experimental(cls.tmp.name)
        cls.round2=Experimental(cls.tmp.name,ROOT/'examples/axis_experimental_models_round2.c')
        cls.stribeck=Experimental(cls.tmp.name,ROOT/'examples/axis_experimental_models_stribeck.c')
        cls.v2=V2(cls.tmp.name)
        cls.u=np.zeros((1,6001));cls.u[0,200:240]=.1;cls.u[0,2000:2040]=-.1
        cls.u[0,4000:4120]=.18;cls.u[0,4300:4340]=-.05
        cls.p=[12000,12500,.0005,.002,260,70,.4,1.2]

    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()

    def test_linear_matches_frozen_v2(self):
        x,v=self.base.simulate(self.u,[6001],[.7],self.p,0)
        xx,vv=self.v2.simulate(self.u,[6001],.0005,[12000,12500,260,70,.4,1.2,.0005,.002])
        np.testing.assert_allclose(x,xx,atol=1e-9);np.testing.assert_allclose(v,vv,atol=1e-9)

    def test_zero_phase_amplitudes_match_linear(self):
        x,v=self.base.simulate(self.u,[6001],[.7],self.p,0)
        xx,vv=self.base.simulate(self.u,[6001],[.7],self.p+[0]*4,3)
        np.testing.assert_array_equal(x,xx);np.testing.assert_array_equal(v,vv)

    def test_constant_spline_matches_coulomb(self):
        p=self.p.copy();p[6:8]=[0,0]
        x,v=self.base.simulate(self.u,[6001],[0],p,0)
        for kind in (2,5):
            xx,vv=self.round2.simulate(self.u,[6001],[0],p[:4]+[260]*5+[70]*5,kind)
            np.testing.assert_allclose(x,xx,atol=1e-10);np.testing.assert_allclose(v,vv,atol=1e-10)

    def test_bristle_equilibrium_remains_at_rest(self):
        p=self.p[:4]+[150,-.6,.4,1.2,5000,.02]
        for kind,pp in ((4,p),(7,p+[3,5,17,-2])):
            x,v=self.round2.simulate(np.zeros((2,2000)),[2000,1500],[.7,1.3],pp,kind)
            np.testing.assert_allclose(x,0,atol=1e-12);np.testing.assert_allclose(v,0,atol=1e-12)

    def test_zero_stribeck_extra_matches_linear(self):
        x,v=self.base.simulate(self.u,[6001],[0],self.p,0)
        xx,vv=self.stribeck.simulate(self.u,[6001],[0],self.p+[0,0,5],8)
        np.testing.assert_allclose(x,xx,atol=1e-10);np.testing.assert_allclose(v,vv,atol=1e-10)

    def test_compression_reduces_motion_and_kernels_do_not_alias(self):
        u=np.zeros((1,401));u[0,10:210]=.1
        x,v=self.base.simulate(u,[401],[0],self.p,0)
        xx,vv=self.round2.simulate(u,[401],[0],self.p+[1,1],6)
        self.assertLess(xx[0,200],x[0,200]);self.assertLess(vv[0,200],v[0,200])

    def test_derivative_does_not_use_padding(self):
        x=np.tile(np.arange(30)*.0005*2,(2,1));x[1,20:]=0
        v=derivative(x,[30,20]);np.testing.assert_allclose(v[0],2)
        np.testing.assert_allclose(v[1,:20],2);np.testing.assert_array_equal(v[1,20:],0)


if __name__=='__main__':unittest.main()
