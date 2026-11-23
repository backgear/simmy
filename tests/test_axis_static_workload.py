import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_fit_controller_workload import Batch
from axis_pc_tune import snapshot

class StaticCandidateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();root=Path(cls.tmp.name);src=Path(__file__).resolve().parents[1]/'examples'
        cls.models={}
        for kind in ['v2','static_v1','stribeck_v1']:
            out=root/kind;out.mkdir();cls.models[kind]=Batch(src/f'axis_command_model_{kind}.c',out)
        cls.params=np.array([13000,13000,220,50,3,2,.0005,.002])
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def simulate(self,kind,u,extra=()):
        return self.models[kind].simulate(np.ascontiguousarray(u[None,:]),np.array([len(u)],dtype=np.int32),np.r_[self.params,list(extra)])
    def test_zero_extra_matches_frozen_v2_exactly(self):
        u=np.random.default_rng(23270).uniform(-.1,.1,20000)
        baseline=self.simulate('v2',u)
        np.testing.assert_array_equal(baseline,self.simulate('static_v1',u,[0]))
        np.testing.assert_array_equal(baseline,self.simulate('stribeck_v1',u,[0,1]))
    def test_extra_threshold_holds_below_breakaway(self):
        u=np.full(1000,.02)
        self.assertGreater(self.simulate('v2',u)[0,-1],0)
        np.testing.assert_array_equal(self.simulate('static_v1',u,[100]),0)
        np.testing.assert_array_equal(self.simulate('stribeck_v1',u,[100,1]),0)
    def test_static_only_leaves_strong_launch_coast_unchanged(self):
        u=np.zeros(3000);u[:40]=.1
        baseline=self.simulate('v2',u)
        np.testing.assert_array_equal(baseline,self.simulate('static_v1',u,[100]))
        self.assertLess(self.simulate('stribeck_v1',u,[100,1])[0,-1],baseline[0,-1])
    def test_current_tuner_refuses_unknown_plant_family(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'model.json';p.write_text(json.dumps(dict(schema='axis-command-model-stribeck-v1')))
            with self.assertRaisesRegex(ValueError,'unsupported plant family'):snapshot(Path(d),p)

if __name__=='__main__':unittest.main()
