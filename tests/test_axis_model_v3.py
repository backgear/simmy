import ast
import json
from pathlib import Path
import sys
import subprocess
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'examples'))
from axis_fit_controller_workload import Batch, NAMES
from axis_model_v3 import legacy, EXTRA
from axis_v3_compare import native_parameters
from axis_v3_chain import Stateful, initial_state, phase_gap
from axis_v3_ramp_preview import ramp_reference


class V3KernelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        cls.models = {}
        for name in ['v2', 'v3', 'v3_slip', 'v3_age']:
            out = Path(cls.tmp.name)/name; out.mkdir()
            cls.models[name] = Batch(ROOT/'examples'/('axis_command_model_'+name+'.c'), out)
        cfg = json.loads((ROOT/'models/axis/sm232ae_tq10_command_v2.json').read_text())
        cls.p = np.array([cfg['parameters'][n] for n in NAMES])
        cls.stateful = Stateful(ROOT/'examples/axis_v3_stateful.c', Path(cls.tmp.name)/'stateful')

    @classmethod
    def tearDownClass(cls): cls.tmp.cleanup()

    def run_model(self, name, effort, extra, dt=.0005):
        u = np.ascontiguousarray(effort, dtype=np.float64).reshape(1, -1)
        lengths = np.array([u.shape[1]], dtype=np.int32)
        p = np.ascontiguousarray(np.r_[self.p, extra]); x = np.zeros_like(u); v = np.zeros_like(u)
        ptr = lambda a: a.ctypes.data
        self.models[name].lib.axis_model(ptr(u), ptr(lengths), 1, u.shape[1], dt, ptr(p), ptr(x), ptr(v))
        return x[0], v[0]

    def test_stateful_adapter_matches_original_batch_kernels(self):
        u = np.repeat(np.random.default_rng(23262).uniform(-.1,.1,80),40)
        for family,name,extra in [(0,'v2',[]),(1,'v3',[350,350,10000,3]),
                (2,'v3_slip',[350,350,10000,.5]),(3,'v3_age',[350,350,10000,3,.5,.05])]:
            expected,ev = self.run_model(name,u,extra)
            params=np.r_[self.p,extra];state=initial_state()
            x,v=self.stateful.run(family,u,params,state)
            np.testing.assert_array_equal(x,expected);np.testing.assert_array_equal(v,ev)
            state=initial_state();a,_=self.stateful.run(family,u[:1001],params,state)
            self.stateful.gap(family,0,params,state)
            b,_=self.stateful.run(family,u[1000:],params,state)
            np.testing.assert_array_equal(np.r_[a[:-1],b],expected)

    def test_stateful_gap_preserves_subsample_clock_remainder(self):
        params=np.r_[self.p,350,350,10000,3,.5,.05];state=initial_state()
        self.stateful.run(3,np.full(20,.1),params,state)
        expected=state.copy();self.stateful.run(3,np.zeros(2),params,expected,dt=.000125)
        self.stateful.gap(3,125,params,state)
        np.testing.assert_array_equal(state,expected)
        with self.assertRaisesRegex(ValueError,'parameter length'):
            self.stateful.run(4,np.zeros(2),params,state)
        with self.assertRaisesRegex(ValueError,'state must'):
            self.stateful.run(3,np.zeros(2),params,state.astype(np.float32))

    def test_disabled_additions_reproduce_v2(self):
        u = np.repeat(np.random.default_rng(23261).uniform(-.2, .2, 300), 40)
        reference, rv = self.run_model('v2', u, [])
        for name, extra in [('v3', [0, 0, 10000, 3]), ('v3_slip', [0, 0, 10000, 0]),
                            ('v3_age', [0, 0, 10000, 3, .5, .05])]:
            x, v = self.run_model(name, u, extra)
            np.testing.assert_allclose(x, reference, rtol=0, atol=1e-11)
            np.testing.assert_allclose(v, rv, rtol=0, atol=1e-10)

    def test_frozen_strength_limit_matches_speed_yield(self):
        u = np.repeat([.1, 0, -.03, 0, .02, 0], 1000)
        reference, _ = self.run_model('v3', u, [350, 350, 10000, 3])
        x, _ = self.run_model('v3_age', u, [350, 350, 10000, 3, 1e100, 1e100])
        np.testing.assert_allclose(x, reference, rtol=0, atol=1e-11)

    def test_zero_command_cannot_create_motion(self):
        for name, extra in [('v3', [350, 600, 100000, 4]), ('v3_slip', [350, 600, 100000, .8]),
                            ('v3_age', [350, 600, 100000, 4, .5, .05])]:
            x, v = self.run_model(name, np.zeros(20000), extra)
            np.testing.assert_array_equal(x, 0); np.testing.assert_array_equal(v, 0)

    def test_long_reversals_remain_finite_and_rest_after_zero_effort(self):
        u = np.r_[np.repeat([.1, -.1, .03, -.02, 0], 400), np.zeros(10000)]
        _, baseline_v = self.run_model('v2', u, [])
        for name, extra in [('v3', [250, 615, 99500, 4.3]), ('v3_slip', [350, 350, 10000, .5])]:
            x, v = self.run_model(name, u, extra)
            self.assertTrue(np.all(np.isfinite(x))); self.assertTrue(np.all(np.isfinite(v)))
            self.assertLess(np.max(abs(v)), 1.2*np.max(abs(baseline_v))+5)
            self.assertLess(np.ptp(x[-1000:]), 1e-10)
            self.assertLess(np.max(abs(v[-1000:])), 1e-8)

    def test_strong_pulse_time_step_convergence(self):
        # These pulses are far from the uncertain breakaway boundary.
        for name, extra in [('v3', [200, 300, 10000, 3]), ('v3_slip', [200, 300, 10000, .5])]:
            final = []
            for dt in [.0005, .00025, .000125]:
                t = np.arange(round(1.5/dt)+1)*dt
                u = np.where((t>=.1)&(t<.14), .1, 0.)
                x, _ = self.run_model(name, u, extra, dt)
                final.append(x[-1])
            self.assertLess(abs(final[2]-final[1]), .01)
            self.assertLess(abs(final[1]-final[0]), .02)

    def test_spring_state_is_not_reset_at_command_changes(self):
        # A continuous replay must differ from restarting the plant at a pulse.
        u = np.r_[np.full(200, .04), np.zeros(1800), np.full(400, -.015), np.zeros(2000)]
        for name, extra in [('v3', [350, 350, 10000, 3]), ('v3_slip', [350, 350, 10000, .5])]:
            full, _ = self.run_model(name, u, extra)
            reset, _ = self.run_model(name, u[2000:], extra)
            self.assertGreater(np.max(abs(full[2000:]-full[2000]-reset)), 1e-5)


class LegacyInputTests(unittest.TestCase):
    def make_input(self, root, gap=False):
        data = [dict(seq=i+(1 if gap and i>=300 else 0), position=0, torque=0,
                     cmd_position=1024 if 250<=i<300 else 0,
                     cmd_velocity=1 if 400<=i<450 else 0, fault_latched=False,
                     excitation=False) for i in range(600)]
        path = root/'frames.json'; path.write_text(json.dumps(data))
        path.with_name('frames.json.meta.json').write_text(json.dumps(dict(completed=True,
            entry_settings='output.polarity=-1\r\n', exit_settings='output.polarity=-1\r\n')))
        return path

    def test_legacy_rejects_guessed_sign_and_missing_effort(self):
        with tempfile.TemporaryDirectory() as d:
            path = self.make_input(Path(d))
            with self.assertRaisesRegex(ValueError, 'polarity'): legacy(path, 1)
            legacy(path, -1)
            path = self.make_input(Path(d), gap=True)
            with self.assertRaisesRegex(ValueError, 'gaps'): legacy(path, -1)


class ToolInterfaceTests(unittest.TestCase):
    def test_state_carry_rejects_unverified_clock_and_encoder_bridges(self):
        self.assertIsNone(phase_gap(None,None,1000,0))
        self.assertEqual(phase_gap(1000,10,1625,11),625)
        for tick,count in [(1000,10),(999,10),(1625,12)]:
            with self.assertRaises(ValueError):phase_gap(1000,10,tick,count)

    def test_ramp_proposal_is_finite_slow_and_returns_to_origin(self):
        for distance in (1,64,128):
            positive=ramp_reference(distance)
            self.assertEqual(len(positive),600)
            self.assertEqual((positive[0],positive[-1]),(0,0))
            self.assertEqual(np.max(positive),distance)
            self.assertLessEqual(np.max(abs(np.diff(positive))),1)
            np.testing.assert_array_equal(ramp_reference(distance,-1),-positive)
        for distance,sign in [(0,1),(129,1),(128,0),(True,1),(128,True)]:
            with self.assertRaises(ValueError):ramp_reference(distance,sign)

    def test_help_parses_without_motion(self):
        for name in ['axis_model_v3.py', 'axis_v3_compare.py', 'axis_v3_chain.py', 'axis_v3_ramp_preview.py']:
            result = subprocess.run([sys.executable, str(ROOT/'examples'/name), '--help'], capture_output=True, text=True)
            self.assertEqual(result.returncode, 0, result.stderr)

    def test_comparer_rejects_unknown_family_and_extra_parameters(self):
        with tempfile.TemporaryDirectory() as d:
            fit = Path(d)
            path = fit/'candidate-model.json'
            path.write_text(json.dumps(dict(family='unknown', parameters={})))
            with self.assertRaisesRegex(ValueError, 'unknown family'): native_parameters(fit)
            parameters = dict.fromkeys(list(NAMES)+EXTRA+['unrecognized_dynamics'], 1.)
            path.write_text(json.dumps(dict(family='speed-yield', parameters=parameters)))
            with self.assertRaisesRegex(ValueError, 'parameter set'): native_parameters(fit)

    def test_fitter_and_comparer_have_no_motion_backend_imports(self):
        forbidden = {'serial', 'axis_recorder', 'axis_adapter', 'axis_pc_tune', 'axis_servo_native'}
        for name in ['axis_model_v3.py', 'axis_v3_compare.py', 'axis_v3_chain.py', 'axis_v3_ramp_preview.py']:
            tree = ast.parse((ROOT/'examples'/name).read_text())
            imports = set()
            for node in ast.walk(tree):
                if isinstance(node, ast.Import): imports.update(n.name.split('.')[0] for n in node.names)
                elif isinstance(node, ast.ImportFrom): imports.add((node.module or '').split('.')[0])
            self.assertFalse(imports & forbidden)


if __name__ == '__main__': unittest.main()
