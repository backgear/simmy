import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_fit_controller_workload import select_blocks,load

class WorkloadInputTests(unittest.TestCase):
    def test_chronological_blocks_exclude_validation_and_repeats(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)
            history=[dict(trial=i,split='search') for i in range(10)]
            history += [dict(trial=10+i,split='search',role='finalist-repeat') for i in range(3)]
            history += [dict(trial=13,split='untouched_validation')]
            (p/'history.json').write_text(json.dumps(history))
            train,dev,excluded=select_blocks(p)
            self.assertEqual(len(train),8);self.assertEqual(len(dev),2);self.assertEqual(len(excluded),4)
            self.assertEqual(train[-1].name,'trial-0007');self.assertEqual(dev[0].name,'trial-0008')
    def fixture(self,p):
        case=p/'trial-0000';case.mkdir()
        (p/'health.json').write_text(json.dumps(dict(hello=dict(tick_hz=1000000))))
        count=np.zeros(1601,dtype='<i8');count[500:]=8
        telemetry=np.zeros((801,2),dtype='<i4');telemetry[150:200,0]=3276;telemetry[:,1]=256
        count.tofile(case/'stream-0.i64le');telemetry.tofile(case/'stream-2.i32le')
        streams={}
        for sid,name in [('0','stream-0.i64le'),('2','stream-2.i32le')]:
            streams[sid]=dict(sha256=hashlib.sha256((case/name).read_bytes()).hexdigest())
        (case/'report.json').write_text(json.dumps(dict(passed=True,state='COMPLETE',streams=streams)))
        descriptors=[dict(stream_id=int(sid),first_sample_tick=123000,period_ticks=500 if sid=='0' else 1000,
                          sample_count=1601 if sid=='0' else 801,**value) for sid,value in streams.items()]
        (case/'get_run.json').write_text(json.dumps(dict(streams=descriptors)))
        (case/'plan.json').write_text(json.dumps(dict(manifest=dict(output_mode='analog_position_servo',analog_enabled=False,
            encoder_sample_period_ticks=500,analog_sample_period_ticks=1000))))
        return case
    def test_actual_effort_not_reference_and_zero_order_hold(self):
        with tempfile.TemporaryDirectory() as d:
            case=self.fixture(Path(d));u,x,_=load(case)
            np.testing.assert_array_equal(u[:300],0)
            np.testing.assert_allclose(u[300:400],3276/32767)
            np.testing.assert_array_equal(u[400:],0)
            self.assertAlmostEqual(x[-1],8*2*np.pi/4096)
    def test_wrong_stream_period_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            case=self.fixture(Path(d));p=case/'get_run.json';v=json.loads(p.read_text())
            v['streams'][0]['period_ticks']=1000;p.write_text(json.dumps(v))
            with self.assertRaisesRegex(ValueError,'stream periods'):load(case)
    def test_wrong_descriptor_length_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            case=self.fixture(Path(d));p=case/'get_run.json';v=json.loads(p.read_text())
            v['streams'][0]['sample_count']-=1;p.write_text(json.dumps(v))
            with self.assertRaisesRegex(ValueError,'raw lengths'):load(case)
    def test_corrupted_stream_rejected(self):
        with tempfile.TemporaryDirectory() as d:
            case=self.fixture(Path(d));(case/'stream-2.i32le').write_bytes(b'corrupt')
            with self.assertRaisesRegex(ValueError,'hash mismatch'):load(case)

if __name__=='__main__':unittest.main()
