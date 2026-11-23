"""ctypes bridge to the real chain stepper in src/simmy.c.

Mirrors SimmyBodyParams/SimmyCouplingParams/SimmyChainParams/SimmyChainState
from ../include/simmy.h field-for-field -- plain fixed-size structs of
doubles/ints, no pointers inside, so this is a direct layout mirror, not a
serialization format. Binds chain_fit_bridge.c's ChainFitSimulate, the one
function that loops the real SimmyChainStepDt over a whole trace per call.

Run ../characterization_fitter/build.sh once before importing this -- it
loads build/chain_fit.so, which that script produces from the actual
src/simmy.c, not a reimplementation.
"""

import ctypes
from pathlib import Path

import numpy as np

SIMMY_MAX_BODIES = 8


class SimmyBodyParams(ctypes.Structure):
    _fields_ = [
        ("inertia", ctypes.c_double),
        ("friction", ctypes.c_double),
        ("stictionLimit", ctypes.c_double),
        ("coulombLevel", ctypes.c_double),
        ("gravityBias", ctypes.c_double),
        ("rippleAmplitude", ctypes.c_double),
        ("ripplePeriodCounts", ctypes.c_double),
        ("frictionBumpCenter", ctypes.c_double),
        ("frictionBumpHalfWidth", ctypes.c_double),
        ("frictionBumpExtra", ctypes.c_double),
    ]


class SimmyCouplingParams(ctypes.Structure):
    _fields_ = [
        ("springK", ctypes.c_double),
        ("springD", ctypes.c_double),
        ("backlashGap", ctypes.c_double),
        ("forceLimit", ctypes.c_double),
    ]


class SimmyChainParams(ctypes.Structure):
    _fields_ = [
        ("bodyCount", ctypes.c_int),
        ("bodies", SimmyBodyParams * SIMMY_MAX_BODIES),
        ("couplings", SimmyCouplingParams * (SIMMY_MAX_BODIES - 1)),
        ("feedbackBody", ctypes.c_int),
        ("subSteps", ctypes.c_int),
    ]


class SimmyBodyState(ctypes.Structure):
    _fields_ = [
        ("position", ctypes.c_double),
        ("velocity", ctypes.c_double),
    ]


class SimmyChainState(ctypes.Structure):
    _fields_ = [
        ("bodies", SimmyBodyState * SIMMY_MAX_BODIES),
    ]


BODY_FIELDS = [name for name, _ in SimmyBodyParams._fields_]
COUPLING_FIELDS = [name for name, _ in SimmyCouplingParams._fields_]

_LIB = None


def _lib():
    global _LIB
    if _LIB is None:
        so_path = Path(__file__).with_name("build") / "chain_fit.so"
        if not so_path.exists():
            raise FileNotFoundError(
                f"{so_path} not found -- run characterization_fitter/build.sh first"
            )
        lib = ctypes.CDLL(str(so_path))
        lib.ChainFitSimulate.argtypes = [
            ctypes.POINTER(SimmyChainParams),
            ctypes.POINTER(SimmyChainState),
            ctypes.POINTER(ctypes.c_double),
            ctypes.c_int,
            ctypes.c_double,
            ctypes.POINTER(ctypes.c_double),
        ]
        lib.ChainFitSimulate.restype = None
        _LIB = lib
    return _LIB


def make_params(bodies, couplings, feedback_body=0, sub_steps=0):
    """bodies/couplings: lists of dicts keyed by the struct field names above
    (missing fields default to 0.0, same as the JSON loader in
    src/simmy_config.c). bodyCount is len(bodies)."""
    params = SimmyChainParams()
    params.bodyCount = len(bodies)
    for i in range(SIMMY_MAX_BODIES):
        b = bodies[i] if i < len(bodies) else {}
        for name in BODY_FIELDS:
            setattr(params.bodies[i], name, float(b.get(name, 0.0)))
    for i in range(SIMMY_MAX_BODIES - 1):
        c = couplings[i] if i < len(couplings) else {}
        for name in COUPLING_FIELDS:
            setattr(params.couplings[i], name, float(c.get(name, 0.0)))
    params.feedbackBody = feedback_body
    params.subSteps = sub_steps
    return params


def simulate(params, initial_state, force, dt_ticks):
    """force: 1D array of length `steps`. Returns an (bodyCount, steps)
    array of every body's continuous position at the end of each tick."""
    force = np.ascontiguousarray(force, dtype=np.float64)
    steps = force.shape[0]
    out = np.zeros((params.bodyCount, steps), dtype=np.float64)
    _lib().ChainFitSimulate(
        ctypes.byref(params),
        ctypes.byref(initial_state),
        force.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
        steps,
        float(dt_ticks),
        out.ctypes.data_as(ctypes.POINTER(ctypes.c_double)),
    )
    return out
