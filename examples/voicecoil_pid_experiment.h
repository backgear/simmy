#ifndef VOICECOIL_PID_EXPERIMENT_H
#define VOICECOIL_PID_EXPERIMENT_H

/* Shared experiment fixture for a closed-loop VERTICAL voice-coil
 * actuator -- NOT a new plant or a production controller claim, same
 * disclaimer examples/induction_foc.h carries. Shared by
 * examples/voicecoil_pid.c (the CSV-producing demo) and
 * tests/test_voicecoil_pid.c (the settling/descent-control regression).
 *
 * Scenario: no mechanical return spring. Gravity alone (SimmyBodyParams.
 * gravityBias -- the existing "vertical-axis-style load" constant-force
 * field; see include/simmy.h) pulls the armature down. There's no
 * anchored-spring field in SimmyBodyParams and this fixture doesn't add
 * one -- gravityBias is already the physically correct model for a
 * constant gravitational load. The voice-coil family is used deliberately
 * (not reluctance): SimmyVoiceCoilChainStep's SIGNED force can push up
 * against gravity AND pull down faster than gravity alone would, which a
 * reluctance solenoid (attraction-only, see include/simmy_solenoid.h)
 * structurally cannot do.
 *
 * Controller: a genuine PID, not just PD. A constant disturbance
 * (gravityBias) needs either integral action or an explicit feedforward
 * term to reach zero steady-state error -- a pure P/D controller settles
 * with permanent sag under gravity (classic "P-only control can't reject
 * a constant load" result). This fixture uses a real integral term with
 * BACK-CALCULATION anti-windup, the same convention
 * examples/induction_foc.h's FocCurrentStepFrame/FocSpeedStep already use
 * in this repo (`raw = Kp*error + integral [+feedforward]`, clamp to
 * `applied`, then `integral += dt*(Ki*error + Kaw*(applied-raw))`) rather
 * than an explicit feedforward term -- partly to demonstrate the integral
 * actually learning to cancel the gravity load (watch
 * VoicecoilPidState.integral converge to roughly
 * resistance*gravityBias/(mount.forcePerNewton*forceConstant), the hold
 * current's IR drop), and partly
 * because a real actuator's load/mass is rarely known precisely enough
 * to feedforward exactly -- integral action is robust to that the way an
 * open-loop feedforward number isn't.
 *
 * Electrical drive mode: VOLTAGE-mode. The PID's output drives
 * SimmyVoiceCoilStep directly as a voltage, so the full R-L electrical
 * dynamics are simulated, not skipped. A current-mode drive (PID output
 * mapped directly to force=Kf*I, no electrical dynamics) is a strict
 * simplification of this and would be a smaller, different example --
 * not built here.
 */
#include "simmy.h"
#include <math.h>

static inline double VoicecoilPidClamp(double value, double limit) {
  return fmax(-limit, fmin(limit, value));
}

typedef struct {
  double kp, ki, kd, ka;   /* position, integral, velocity-derivative, anti-windup gains */
  double voltageLimit;     /* V, symmetric drive voltage ceiling */
  double dtSeconds;        /* controller/plant tick, held fixed for this fixture */
  double integral;         /* V, running integral state */
} VoicecoilPidState;

static inline void VoicecoilPidInit(VoicecoilPidState *s, double kp, double ki, double kd,
                                     double ka, double voltageLimit, double dtSeconds) {
  s->kp = kp; s->ki = ki; s->kd = kd; s->ka = ka;
  s->voltageLimit = voltageLimit; s->dtSeconds = dtSeconds; s->integral = 0;
}

/* Position/velocity in COUNTS and counts/tick -- the same feedback
 * convention every other demo controller in examples/ uses (e.g.
 * examples/basic_two_mass.c's DemoPdController). Derivative acts on
 * MEASURED velocity, not d(error)/dt, avoiding derivative kick on a step
 * command -- same choice DemoPdController makes. Returns the applied
 * voltage (V), already clamped to +-voltageLimit. */
static inline double VoicecoilPidStep(VoicecoilPidState *s, double targetCounts,
                                       double actualCounts, double actualVelocityCounts) {
  double error = targetCounts - actualCounts;
  double raw = s->kp * error + s->integral - s->kd * actualVelocityCounts;
  double applied = VoicecoilPidClamp(raw, s->voltageLimit);
  s->integral += s->dtSeconds * (s->ki * error + s->ka * (applied - raw));
  return applied;
}

/* Sample plant: see docs/motors/solenoid.md's voice-coil table for the
 * motor; inertia/friction are in the chain's own abstract units (NOT kg/
 * kg-per-second -- see include/simmy.h's SimmyBodyParams comment), picked
 * heavier/more damped than a light bench scale for the same reason the
 * other solenoid examples/tests are (explicit-Euler mechanical
 * integration needs it at this voltage/force scale; see
 * examples/solenoid_actuators.c). gravityBias=1.2N is chosen so the
 * steady hold current is exactly 1.0A (gravityBias/forceConstant) --
 * convenient for reading the plots/tests, not a claim about any real
 * actuator's mass. */
typedef struct {
  SimmyChainParams chain;
  SimmyChainState mechanics;
  SimmyLinearMount mount;
  SimmyVoiceCoilParams motor;
  SimmyVoiceCoilState electrical;
  VoicecoilPidState pid;
} VoicecoilPidRig;

/* Gains tuned by hand for THIS plant via step-response inspection -- not
 * tuned for any other configuration (same caveat basic_two_mass.c's
 * DemoPdController carries). */
static inline void VoicecoilPidRigInit(VoicecoilPidRig *rig) {
  rig->motor = (SimmyVoiceCoilParams){.resistance = 3.0, .inductance = .0006, .forceConstant = 1.2};
  SimmyVoiceCoilInit(&rig->electrical);
  rig->mount = (SimmyLinearMount){.metersPerCount = 1e-6, .forcePerNewton = 1, .tickSeconds = .0001};
  rig->chain = (SimmyChainParams){0};
  rig->chain.bodyCount = 1;
  rig->chain.subSteps = 1;
  rig->chain.bodies[0].inertia = 20;
  rig->chain.bodies[0].friction = 1;
  rig->chain.bodies[0].gravityBias = 1.2; /* N -- see the comment above */
  SimmyChainInit(&rig->mechanics, &rig->chain);
  VoicecoilPidInit(&rig->pid, .02, .05, .3, 2.0, 24.0, rig->mount.tickSeconds);
}

/* One controller+plant tick. Returns the applied voltage (useful for
 * logging); the plant step failing (non-finite input) is treated as the
 * caller's problem, same as every other ChainStep-wrapping example here. */
static inline double VoicecoilPidTick(VoicecoilPidRig *rig, double targetCounts) {
  int32_t position = SimmyChainFeedbackPositionCounts(&rig->mechanics, &rig->chain);
  int32_t velocity = SimmyChainFeedbackVelocityCounts(&rig->mechanics, &rig->chain);
  double voltage = VoicecoilPidStep(&rig->pid, targetCounts, position, velocity);
  SimmyVoiceCoilChainStep(&rig->mechanics, &rig->chain, &rig->electrical, &rig->motor,
                           &rig->mount, voltage, rig->mount.tickSeconds);
  return voltage;
}

/* Step schedule shared by the example and the test: hold at the bottom,
 * lift, then command back down -- see each caller for the exact timing. */
static inline double VoicecoilPidTarget(double timeSeconds) {
  if (timeSeconds < .5) return 0;
  if (timeSeconds < 2.5) return 2000;
  return 0;
}

#endif /* VOICECOIL_PID_EXPERIMENT_H */
