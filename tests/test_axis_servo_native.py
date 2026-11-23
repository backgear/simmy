import csv
import ctypes
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'examples'))
from axis_model_fit_v2 import Model


@unittest.skipUnless(shutil.which('cc'),'requires native C compiler')
class StatefulAdapterTests(unittest.TestCase):
    def test_stateful_adapter_matches_frozen_batch_kernel(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);src=p/'step.c';so=p/'step.so'
            src.write_text('#include "axis_command_step.h"\nvoid run(const double*u,int n,const double*p,double*x,double*v){'
                'AxisCommandState s={0};x[0]=v[0]=0;for(int i=1;i<n;i++){axis_command_step(&s,u[i-1],.0005,p);x[i]=s.x;v[i]=s.v;}}')
            subprocess.run(['cc','-shared','-fPIC','-O2','-I',str(ROOT/'examples'),str(src),'-lm','-o',str(so)],check=True)
            lib=ctypes.CDLL(str(so));lib.run.argtypes=[ctypes.c_void_p,ctypes.c_int,ctypes.c_void_p,ctypes.c_void_p,ctypes.c_void_p];lib.run.restype=None
            u=np.zeros((1,6001));u[0,100:140]=.1;u[0,1800:1880]=-.18;u[0,4000:4120]=.08;u[0,4250:4270]=-.03
            params=np.array([12969,13096,222,50,3.4,1.93,.000457,.001945],dtype=np.float64)
            expected_x,expected_v=Model(d).simulate(u,[6001],.0005,params)
            x=np.zeros(6001);v=np.zeros(6001)
            lib.run(u.ctypes.data,6001,params.ctypes.data,x.ctypes.data,v.ctypes.data)
            np.testing.assert_allclose(x,expected_x[0],atol=1e-12);np.testing.assert_allclose(v,expected_v[0],atol=1e-12)


@unittest.skipUnless(shutil.which('cc') and (ROOT.parent/'analog-servo/firmware/src/Autotune.c').exists(),
                     'requires external analog-servo checkout and C compiler')
class RealControllerNativeTests(unittest.TestCase):
    def test_actual_autotune_and_control_use_encoder_delta(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'run'
            subprocess.run([sys.executable,str(ROOT/'examples/axis_servo_native.py'),
                '--controller',str(ROOT.parent/'analog-servo'),'--model',str(ROOT/'models/axis/sm232ae_tq10_command_v2.json'),
                '--output',str(out),'--max-ticks','2000','--hold-ticks','50','--no-plot'],check=True,capture_output=True)
            result=json.loads((out/'result.json').read_text());self.assertEqual(result['result'],1)
            self.assertTrue(result['simulated_only']);self.assertGreater(result['velocity_kp_q8'],0)
            with (out/'telemetry.csv').open() as f:rows=list(csv.DictReader(f))
            previous=0
            for row in rows:
                position=int(row['position_count'])
                self.assertEqual(int(row['velocity_count_per_tick']),position-previous);previous=position
                self.assertLessEqual(abs(int(row['effort_code'])),6553)
            self.assertIn('control',{r['phase'] for r in rows})
            self.assertFalse(json.loads((out/'manifest.json').read_text())['hardware_comparison_done'])


if __name__=='__main__':unittest.main()
