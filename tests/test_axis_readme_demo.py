"""Optional integration regressions for the sibling-controller demo, sim only."""
import json
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[1]
UPSTREAM=ROOT.parent/'analog-servo-rls/firmware'

@unittest.skipUnless(shutil.which('cc') and (UPSTREAM/'src/RLS.c').exists(),
                     'optional sibling RLS/controller sources unavailable')
class DemoParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp=tempfile.TemporaryDirectory();out=Path(cls.tmp.name);binary=out/'demo'
        subprocess.run(['make','lib'],cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
        command=['cc','-std=gnu11','-O2','-I'+str(ROOT/'include'),'-I'+str(ROOT/'src'),'-I'+str(UPSTREAM/'include'),
                 str(ROOT/'examples/rls_axis_command_v2/main.c')]
        command += [str(UPSTREAM/'src'/name) for name in ['Control.c','RLS.c','GainGlide.c']]
        subprocess.run(command+[str(ROOT/'build/libsimmy.a'),'-lm','-o',str(binary)],check=True)
        cls.data={}
        for name,flag in [('legacy','--readme-demo-legacy'),('no-reference','--readme-demo-no-reference'),('corrected','--readme-demo')]:
            target=out/(name+'.json')
            subprocess.run([str(binary),flag,str(target),'64000','4096','300','3276','16','1024','500','28','250'],
                           cwd=ROOT,check=True,stdout=subprocess.DEVNULL)
            cls.data[name]=json.loads(target.read_text())
    @classmethod
    def tearDownClass(cls):cls.tmp.cleanup()
    def test_feedback_is_quantized_position_difference(self):
        for name in ['corrected','no-reference']:
            d=self.data[name]
            for i,f in enumerate(d):self.assertEqual(f['velocity'],f['position']-d[i-1]['position'] if i else 0)
        self.assertGreater(sum(f['velocity']!=f['encoder_delta'] for f in self.data['legacy']),50)
    def test_reference_schedule_unchanged_no_fitted_time_shift(self):
        for name,d in self.data.items():
            self.assertEqual([(f['seq'],f['cmd_position'],f['cmd_velocity']) for f in d],
                             [(f['seq'],f['cmd_position'],f['cmd_velocity']) for f in self.data['legacy']])
            for f in d:
                self.assertAlmostEqual(f['t'],f['seq']/1000)
                self.assertEqual(f['applied_velocity_reference'],f['cmd_velocity'] if name=='corrected' else 0)
    def test_missing_reference_creates_ramp_lag(self):
        def lag(name):
            d=[f for f in self.data[name] if f['phase'].startswith('ramp-')]
            commanded=next(f['seq'] for f in d if f['cmd_position']>=2048)
            actual=next(f['seq'] for f in d if f['position']>=2048)
            return actual-commanded
        self.assertGreaterEqual(lag('no-reference'),10)
        self.assertLessEqual(abs(lag('corrected')),4)
    def test_integral_units_and_unsaturated_decomposition(self):
        for f in self.data['corrected']:
            self.assertLessEqual(abs(f['torque']),3276)
            if abs(f['unclamped_candidate_torque'])<3276:
                self.assertAlmostEqual(f['integral']-f['integral_before'],f['velocity_error_q8']*4096/65536)
                self.assertLessEqual(abs(f['torque_minus_stored_integral']-f['proportional']),.50001)

if __name__=='__main__':unittest.main()
