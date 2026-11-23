import ctypes as ct
import hashlib
import json
from pathlib import Path
import shutil
import sys
import tempfile
import unittest
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'examples'))
from axis_pc_tune import Config,SimBackend,snapshot,score,search,expanded_search,repeat_finalists
from axis_servo_config import DEFAULT,validate_servo,workload

class SearchTests(unittest.TestCase):
    def test_backend_independent_search_and_bounds(self):
        seen=[]
        def evaluate(c):
            validate_servo(c);seen.append(dict(c))
            return dict(cost=abs(c['velocity_kp_q8']-32000)/32000+abs(c['velocity_ki_q8']-512)/512,
                        motion_adequate=True)
        best,metrics=search(evaluate,dict(DEFAULT),2)
        self.assertEqual(best['velocity_kp_q8'],32000);self.assertEqual(best['velocity_ki_q8'],512)
        self.assertEqual(metrics['cost'],0);self.assertTrue(all(c['torque_limit_code']==3276 for c in seen))
    def test_no_stationary_campaign(self):
        with self.assertRaisesRegex(RuntimeError,'inadequate motion'):
            search(lambda c:dict(cost=1,motion_adequate=False),dict(DEFAULT))
    def test_expansion_and_refinement_use_bounded_integer_gains(self):
        seen=[]
        def evaluate(c):
            validate_servo(c);seen.append(dict(c))
            return dict(cost=abs(c['position_kp_q8']-24)/24 + abs(c['velocity_kp_q8']-96000)/96000 + abs(c['velocity_ki_q8']-1536)/1536,motion_adequate=True)
        best,metrics=expanded_search(evaluate,dict(DEFAULT))
        self.assertTrue(any(c['position_kp_q8']>16 for c in seen))
        self.assertTrue(any(c['position_kp_q8']==20 for c in seen))
        self.assertLess(metrics['cost'],.25)
        self.assertTrue(all(c['position_kp_q8']<=32 and c['torque_limit_code']==3276 for c in seen))
    def test_finalists_are_distinct_and_repeated_interleaved(self):
        history=[]
        for i,p in enumerate((4,8,16)):
            c=dict(DEFAULT,position_kp_q8=p)
            history.append(dict(split='search',gains=c,metrics=dict(cost=i)))
        seen=[]
        def evaluate(c,role):
            self.assertEqual(role,'finalist-repeat');seen.append(c['position_kp_q8'])
            return dict(cost=abs(c['position_kp_q8']-8),motion_adequate=True)
        best,metrics,summary=repeat_finalists(evaluate,history,3)
        self.assertEqual(best['position_kp_q8'],8)
        self.assertEqual(len(seen),9)
        self.assertEqual(set(seen[:3]),{4,8,16})
        self.assertTrue(all(len(x['costs'])==3 for x in summary))
    def test_shared_source_identity(self):
        p=ROOT/'characterization_rig/lib/axis_pid';v=json.loads((p/'source_id.json').read_text())
        hashes={n:hashlib.sha256((p/n).read_bytes()).hexdigest() for n in v['files']}
        self.assertEqual(hashes,v['files'])
        self.assertEqual(hashlib.sha256(json.dumps(hashes,sort_keys=True).encode()).hexdigest(),v['sha256'])
        self.assertIn(v['sha256'],(p/'source_id.h').read_text())
    def test_workload_is_positions_and_zero_endpoints(self):
        refs=workload();self.assertEqual(refs[0],0);self.assertEqual(refs[-1],0)
        self.assertEqual(set(refs),{0,256,-256})

@unittest.skipUnless(shutil.which('cc'),'requires C compiler')
class NativeControllerTests(unittest.TestCase):
    def test_feedback_motion_and_latching_interlock(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d);snapshot(p,ROOT/'models/axis/sm232ae_tq10_command_v2.json');backend=SimBackend(p)
            case=p/'trial';case.mkdir()
            arrays=backend.trial(case,dict(DEFAULT),workload());metrics=score(*arrays,256,3276)
            self.assertTrue(metrics['motion_adequate']);self.assertGreater(metrics['excursion_counts'],256)
            self.assertLessEqual(metrics['peak_effort_code'],3276)
            lib=backend.lib;lib.axis_pid_init.argtypes=[ct.POINTER(Config),ct.c_int32];lib.axis_pid_init.restype=ct.c_bool
            lib.axis_pid_tick.argtypes=[ct.c_int32,ct.c_int32,ct.POINTER(ct.c_int32)];lib.axis_pid_tick.restype=ct.c_bool
            c=Config(**DEFAULT);u=ct.c_int32(123)
            self.assertTrue(lib.axis_pid_init(ct.byref(c),0))
            self.assertFalse(lib.axis_pid_tick(2048,0,ct.byref(u)));self.assertEqual(u.value,0)
            self.assertFalse(lib.axis_pid_tick(0,0,ct.byref(u)));self.assertEqual(u.value,0)
            self.assertTrue(lib.axis_pid_init(ct.byref(c),0));self.assertTrue(lib.axis_pid_tick(0,0,ct.byref(u)))
            self.assertEqual(u.value,0)

if __name__=='__main__':unittest.main()
