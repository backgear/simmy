"""Synthetic self-test for fit_chain.py: generate ground truth from a real
SimmyChainParams via chain_bridge (the same forward model the fitter
itself uses), fit it back, and check recovery -- same invariant
tests/test_axis_identification.py already checks (held-out data never
leaks into the fitted parameters), with a looser tolerance than that
module's 5%, since this is the deliberately-quick single-pass path (see
characterization_fitter/README.md)."""

import json
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import chain_bridge  # noqa: E402
import fit_chain as fc  # noqa: E402

RECOVERY_TOLERANCE = 0.25  # loose: see README.md's "good enough, quick" scoping.


def _build_library_if_needed():
    so_path = ROOT / "build" / "chain_fit.so"
    if not so_path.exists():
        subprocess.run([str(ROOT / "build.sh")], check=True, cwd=str(ROOT))


def _generate_two_body_data(true_bodies, true_couplings, steps=6000, seed=1):
    params = chain_bridge.make_params(true_bodies, true_couplings, feedback_body=0, sub_steps=20)
    state = chain_bridge.SimmyChainState()
    rng = np.random.default_rng(seed)
    force = np.zeros(steps)
    t = 0
    while t < steps:
        level = rng.uniform(-40, 40)
        hold = rng.integers(8, 40)
        force[t : t + hold] = level
        t += hold
    positions = chain_bridge.simulate(params, state, force, 1.0)
    data = {
        "time": np.arange(steps, dtype=np.float64),
        "force": force,
        "motor_position": positions[0],
        "load_position": positions[1],
    }
    return data


def _two_body_spec():
    return {
        "bodies": [
            {"inertia": {"guess": 15, "min": 1, "max": 200}, "friction": {"guess": 0.1, "min": 0, "max": 1}},
            {"inertia": {"guess": 10, "min": 1, "max": 200}, "friction": {"guess": 0.05, "min": 0, "max": 1}},
        ],
        "couplings": [
            {
                "springK": {"guess": 5, "min": 0.1, "max": 50},
                "springD": {"guess": 2, "min": 0, "max": 20},
                "backlashGap": {"guess": 1, "min": 0, "max": 5},
            },
        ],
        "encoders": [
            {"column": "motor_position", "body": 0},
            {"column": "load_position", "body": 1},
        ],
        "feedbackBody": 0,
        "subSteps": 20,
    }


class TwoBodyRecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        _build_library_if_needed()
        cls.true_bodies = [{"inertia": 20, "friction": 0.05}, {"inertia": 6, "friction": 0.02}]
        cls.true_couplings = [{"springK": 3.0, "springD": 1.0, "backlashGap": 0.5}]
        cls.data = _generate_two_body_data(cls.true_bodies, cls.true_couplings)
        cls.spec = _two_body_spec()
        _, cls.free = fc.load_fit_spec_from_dict(cls.spec)
        cls.overrides, cls.result = fc.run_fit_once(
            cls.spec, cls.free, cls.data, dt_ticks=1.0, encoders=cls.spec["encoders"]
        )

    def test_smooth_parameters_recovered_within_tolerance(self):
        bodies, couplings = fc.materialize(self.spec, self.overrides)
        checks = [
            (bodies[0]["inertia"], self.true_bodies[0]["inertia"]),
            (bodies[0]["friction"], self.true_bodies[0]["friction"]),
            (bodies[1]["inertia"], self.true_bodies[1]["inertia"]),
            (bodies[1]["friction"], self.true_bodies[1]["friction"]),
            (couplings[0]["springK"], self.true_couplings[0]["springK"]),
            (couplings[0]["springD"], self.true_couplings[0]["springD"]),
        ]
        for fitted, true in checks:
            self.assertLess(
                abs(fitted - true),
                RECOVERY_TOLERANCE * abs(true),
                f"fitted={fitted} true={true} outside {RECOVERY_TOLERANCE:.0%} tolerance",
            )

    def test_validate_on_held_out_data_does_not_touch_the_fit(self):
        """There's no train/validation split inside run_fit_once itself --
        `validate` always takes a *separate* data file (see README.md) --
        so the real invariant here is structural: running validation_rmse
        against a second, independently-generated dataset must not change
        the already-fitted parameters. Confirms that by checking they're
        byte-identical before and after, same spirit as
        tests/test_axis_identification.py's leak check even though the
        split is between files here rather than within one."""
        held_out = _generate_two_body_data(self.true_bodies, self.true_couplings, seed=99)
        before_bodies, before_couplings = fc.materialize(self.spec, self.overrides)
        fc.validation_rmse(self.spec, self.overrides, held_out, dt_ticks=1.0, encoders=self.spec["encoders"])
        after_bodies, after_couplings = fc.materialize(self.spec, self.overrides)
        self.assertEqual(before_bodies, after_bodies)
        self.assertEqual(before_couplings, after_couplings)

    def test_fit_is_deterministic(self):
        overrides_again, _ = fc.run_fit_once(
            self.spec, self.free, self.data, dt_ticks=1.0, encoders=self.spec["encoders"]
        )
        bodies_a, couplings_a = fc.materialize(self.spec, self.overrides)
        bodies_b, couplings_b = fc.materialize(self.spec, overrides_again)
        self.assertEqual(bodies_a, bodies_b)
        self.assertEqual(couplings_a, couplings_b)


class RefineComparisonTests(unittest.TestCase):
    """Exercises the --seed CLI path end to end (it's cheap to run as a
    subprocess at this problem size) and checks the report shape: both
    starting points are run and reported, nothing is silently preferred."""

    @classmethod
    def setUpClass(cls):
        _build_library_if_needed()

    def test_seed_runs_both_starts_and_reports_a_choice(self):
        with __import__("tempfile").TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            true_bodies = [{"inertia": 20, "friction": 0.05}, {"inertia": 6, "friction": 0.02}]
            true_couplings = [{"springK": 3.0, "springD": 1.0, "backlashGap": 0.5}]
            data = _generate_two_body_data(true_bodies, true_couplings, steps=4000, seed=2)
            spec = _two_body_spec()
            spec_path = tmp / "spec.json"
            data_path = tmp / "data.csv"
            with open(spec_path, "w") as f:
                json.dump(spec, f)
            with open(data_path, "w") as f:
                f.write("time,force,motor_position,load_position\n")
                for i in range(len(data["time"])):
                    f.write(
                        f"{data['time'][i]},{data['force'][i]},"
                        f"{data['motor_position'][i]},{data['load_position'][i]}\n"
                    )

            first_out = tmp / "first"
            subprocess.run(
                [sys.executable, str(ROOT / "fit_chain.py"), "fit", str(spec_path), str(data_path), "--output", str(first_out)],
                check=True,
            )

            second_out = tmp / "second"
            subprocess.run(
                [
                    sys.executable,
                    str(ROOT / "fit_chain.py"),
                    "fit",
                    str(spec_path),
                    str(data_path),
                    "--output",
                    str(second_out),
                    "--seed",
                    str(first_out / "fit.json"),
                ],
                check=True,
            )

            with open(second_out / "fit-report.json") as f:
                report = json.load(f)
            comparison = report["refine_comparison"]
            self.assertIn("from_past_solution", comparison)
            self.assertIn("from_fresh_guess", comparison)
            self.assertIn(comparison["chosen"], ("from_past_solution", "from_fresh_guess"))


if __name__ == "__main__":
    unittest.main()
