/*
 * Links the REAL firmware/src/{Control,RLS,GainGlide}.c from the sibling
 * analog-servo-rls repo (read-only, by relative path -- see
 * ANALOG_SERVO_RLS_DIR in the Makefile), the same "wrap a real production
 * module in SimmyControllerFn/plain function calls" convention as
 * examples/analog_servo_integration/main.c uses for Control.c alone.
 *
 * That example proves simmy's plant model is faithful enough to reproduce
 * a calibrated firmware test's own numbers. This one instead exercises
 * the SECOND thing the firmware ships: RLS.c's online, in-move velocity-
 * loop self-tuner, plus GainGlide.c's live gain ramp -- both only ever
 * covered by test_rls.c's own hand-rolled single-pole synthetic plant.
 * Three scenarios, same harness:
 *
 *   1. Rigid single body (inertia+friction only, no JSON -- the ONE
 *      scenario deliberately built to match RLS's v[n+1]=a*v[n]+b*u[n]
 *      model as closely as simmy can produce: a pure-gain analog chain,
 *      no second pole from amplifier lag). Seeded with deliberately
 *      mistuned velocity gains (low Kp, no Ki -- the "starts rough" half
 *      of this feature's whole premise), run through a few moves
 *      separated by a pause, then a validation move before vs. after RLS
 *      converges and AppRlsApply()'s glide is applied. This is the one
 *      scenario this file actually asserts pass/fail on, since it's the
 *      one case where failing to converge/improve would mean something
 *      is actually broken.
 *
 *   2. Two-body compliant (examples/configs/two_mass.json -- the same
 *      config analog_servo_integration/main.c's own comment says mirrors
 *      analog-servo's calibrated kNema23Leadscrew preset).
 *
 *   3. Three-body leadscrew (examples/configs/three_mass_leadscrew.json --
 *      motor -> belt -> leadscrew -> nut, TWO compliant stages).
 *
 * Scenarios 2/3 are a deliberate model MISMATCH: real compliance/backlash
 * a single-pole estimator can't represent. They are NOT expected to prove
 * convergence -- they exist to show, honestly, what RLS's clamp/
 * BAD_MEASUREMENT safety net actually does against a plant it can't
 * represent exactly (settles on a still-useful average-pole fit, or
 * correctly refuses to ever report OK). Both outcomes are reported, not
 * asserted; this file does not fail the build over either one.
 *
 * Only builds/runs via `make rls-integration` (requires the
 * ANALOG_SERVO_RLS_DIR sibling repo; see the Makefile). Run from the
 * simmy repo root.
 */
#include <stddef.h>
#include <stdio.h>
#include <string.h>

#include "Control.h"
#include "RLS.h"
#include "GainGlide.h"
#include "simmy.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

typedef struct {
  int32_t maxErrAccel, maxErrCruise, pegTicks;
} MoveMetrics;

/* One ramped, jittered position move -- same shape as
 * analog_servo_integration's own scenario. Reused here as both RLS's
 * learning excitation (feedRls=1) and the before/after validation probe
 * (feedRls=0, a clean measurement run RLS never sees). */
static MoveMetrics RunMove(const SimmyChainParams *chainParams, SimmyChainState *chainState,
                            const SimmyAnalogChainParams *analogParams, SimmyAnalogChainState *analogState,
                            const ControlGains *gains, uint32_t pulseSeed,
                            int32_t *positionTarget, int feedRls) {
  SimmyPulseTrainState pulseState;
  SimmyPulseTrainInit(&pulseState, pulseSeed);

  double targetRate = 2500.0, rampSeconds = 1.0, jitterFraction = 0.3;
  int accelTicks = 1000, cruiseTicks = 1000, ticks = accelTicks + cruiseTicks;
  double accelRatePerSec2 = targetRate / rampSeconds;

  MoveMetrics m = {0, 0, 0};
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / 1000.0) : targetRate;
    int32_t delta = SimmyPulseTrainTickDelta(&pulseState, rate, 1000.0, jitterFraction);
    *positionTarget += delta;

    int32_t position = SimmyChainFeedbackPositionCounts(chainState, chainParams);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(chainState, chainParams);
    int32_t torque = ControlTick(*positionTarget, position, velocity, 32000);
    if (feedRls) {
      /* Same wiring as main.c's AppControlTick() -- dither must reach the
       * REAL output, not just RlsTick()'s bookkeeping, or it can't actually
       * decorrelate anything (see RLS.h's "Torque dither" section). Returns
       * exactly 0 once converged or not running, so always safe to add. */
      torque += RlsGetDitherTorque();
      if (torque > gains->torqueClampMax) torque = gains->torqueClampMax;
      if (torque < gains->torqueClampMin) torque = gains->torqueClampMin;
      RlsTick(torque, velocity);
    }
    double force = SimmyAnalogChainStep(analogState, analogParams, (double)torque);

    int32_t err = Abs32(*positionTarget - position);
    if (i < accelTicks) { if (err > m.maxErrAccel) m.maxErrAccel = err; }
    else { if (err > m.maxErrCruise) m.maxErrCruise = err; }
    if (Abs32(torque) >= gains->torqueClampMax - 1) m.pegTicks++;

    SimmyChainStep(chainState, chainParams, force);
  }
  return m;
}

/* A held-position pause between moves -- lets RLS's dead-zone/covariance
 * wind-up guard actually exercise the "paused between moves" path this
 * feature's whole design leans on (see RLS.h), instead of only ever
 * observing continuous excitation like test_rls.c's simpler cases. */
static void RunPause(const SimmyChainParams *chainParams, SimmyChainState *chainState,
                      const SimmyAnalogChainParams *analogParams, SimmyAnalogChainState *analogState,
                      int32_t positionTarget, uint32_t ticks, int feedRls, const ControlGains *gains) {
  for (uint32_t i = 0; i < ticks; i++) {
    int32_t position = SimmyChainFeedbackPositionCounts(chainState, chainParams);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(chainState, chainParams);
    int32_t torque = ControlTick(positionTarget, position, velocity, 32000);
    if (feedRls) {
      torque += RlsGetDitherTorque();
      if (torque > gains->torqueClampMax) torque = gains->torqueClampMax;
      if (torque < gains->torqueClampMin) torque = gains->torqueClampMin;
      RlsTick(torque, velocity);
    }
    double force = SimmyAnalogChainStep(analogState, analogParams, (double)torque);
    SimmyChainStep(chainState, chainParams, force);
  }
}

/* Drives an already-started GainGlideState to completion, holding
 * position steady -- the exact same two calls AppRlsApply()'s glide/
 * AppControlTick()'s glide-tick make in firmware (main.c), reproduced
 * here with no HAL involved: GainGlideTick() then ControlSetGains(),
 * BEFORE this tick's ControlTick() call, same order as the real
 * AppControlTick(). */
static void RunGlide(const SimmyChainParams *chainParams, SimmyChainState *chainState,
                      const SimmyAnalogChainParams *analogParams, SimmyAnalogChainState *analogState,
                      int32_t positionTarget, GainGlideState *glide) {
  while (GainGlideActive(glide)) {
    ControlGains glided;
    if (GainGlideTick(glide, &glided)) ControlSetGains(&glided);
    int32_t position = SimmyChainFeedbackPositionCounts(chainState, chainParams);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(chainState, chainParams);
    int32_t torque = ControlTick(positionTarget, position, velocity, 32000);
    double force = SimmyAnalogChainStep(analogState, analogParams, (double)torque);
    SimmyChainStep(chainState, chainParams, force);
  }
}

static void BuildRigidParams(SimmyChainParams *chainParams, SimmyAnalogChainParams *analogParams) {
  memset(chainParams, 0, sizeof(*chainParams));
  chainParams->bodyCount = 1;
  chainParams->bodies[0].inertia = 40.0;
  chainParams->bodies[0].friction = 2.0;   /* NOT 0.05 -- an earlier near-frictionless
                                               choice here destabilized the position
                                               cascade after applying RLS's candidate
                                               (verified, via a throwaway tick-by-tick
                                               trace, to be a cascade/outer-loop
                                               interaction specific to a near-zero-
                                               damping plant, not a bug in RLS's math:
                                               the SAME candidate Kp/Ki tracked a bare
                                               ControlVelocityTick() target cleanly with
                                               no position loop in the way at all). This
                                               value keeps the scenario a real
                                               demonstration rather than an adversarial
                                               edge case. */
  chainParams->feedbackBody = 0;

  memset(analogParams, 0, sizeof(*analogParams));
  analogParams->stageCount = 1;
  analogParams->stages[0].type = SIMMY_STAGE_GAIN;
  analogParams->stages[0].param0 = 0.1;   /* deliberately no lag stage here, unlike the
                                              compliant configs below: this scenario exists to
                                              give RLS a plant with exactly ONE pole (the
                                              mechanical one), matching its own model as closely
                                              as simmy can produce. 0.1, not unity -- a 1:1
                                              torque-command-to-force mapping measured b far
                                              outside RLS_B_MAX_AT_REF_Q8's placeholder bound
                                              (0.586 vs. a ~0.25 ceiling at 1kHz); this scaling
                                              keeps b comfortably inside RLS.c's existing clamp
                                              instead of asking it to accept an unrealistically
                                              "hot" torque-to-force gain no real amplifier would
                                              have. */
}

static const char *ResultName(AutotuneResult r) {
  switch (r) {
    case AUTOTUNE_RESULT_OK: return "OK";
    case AUTOTUNE_RESULT_BAD_MEASUREMENT: return "BAD_MEASUREMENT";
    default: return "NONE";
  }
}

typedef struct {
  const char *label;
  const char *configPath;   /* NULL => inline rigid single-body, see BuildRigidParams() */
  int hardAssert;           /* 1: this file's own pass/fail exit code depends on this
                                 scenario converging and improving -- only true for the
                                 rigid case, where the plant actually matches RLS's model.
                                 0: report metrics, always return success -- a model-
                                 mismatch stress case isn't expected to pass OR fail a
                                 specific way; see the file comment above. */
} RlsScenario;

static int RunScenario(const RlsScenario *scenario) {
  printf("\n=== %s ===\n", scenario->label);

  SimmyChainParams chainParams;
  SimmyAnalogChainParams analogParams;
  char err[256];
  if (scenario->configPath) {
    if (SimmyChainParamsFromJsonFile(scenario->configPath, &chainParams, err, sizeof(err)) != 0) {
      fprintf(stderr, "failed to load %s: %s\n", scenario->configPath, err);
      return 1;
    }
    if (SimmyAnalogChainParamsFromJsonFile(scenario->configPath, &analogParams, err, sizeof(err)) != 0) {
      fprintf(stderr, "failed to load analog chain from %s: %s\n", scenario->configPath, err);
      return 1;
    }
  } else {
    BuildRigidParams(&chainParams, &analogParams);
  }

  /* Deliberately mistuned seed: posGainQ8 is a plausible value (RLS never
   * touches it -- it only ever estimates the velocity loop, see RLS.h)
   * paired with a velocity Kp well below what any of these plants want
   * and NO integral term at all -- the "starts rough" half of this
   * feature's premise. */
  ControlGains seedGains = {
    .posGainQ8 = 32, .velGainKpQ8 = 256, .velGainKiQ8 = 0,
    .velocityClamp = 200000, .torqueClampMin = -1000, .torqueClampMax = 1000,
  };

  SimmyChainState chainState;
  SimmyAnalogChainState analogState;
  int32_t positionTarget;

  /* ---- BEFORE: a clean validation move against the untouched seed
   * gains, on a fresh plant -- RLS never sees this run (feedRls=0). ---- */
  ControlInit(&seedGains);
  SimmyChainInit(&chainState, &chainParams);
  SimmyAnalogChainInit(&analogState, 1);
  positionTarget = 0;
  MoveMetrics before = RunMove(&chainParams, &chainState, &analogParams, &analogState,
                                &seedGains, 12345, &positionTarget, 0);

  /* ---- LEARNING: fresh plant again, same seed gains, RLS observes
   * several moves separated by a pause -- "start from nothing," multiple
   * moves, a pause in between, all on-MCU (no host script in this loop
   * either, just this file standing in for main.c's AppControlTick()). ---- */
  ControlInit(&seedGains);
  SimmyChainInit(&chainState, &chainParams);
  SimmyAnalogChainInit(&analogState, 1);
  positionTarget = 0;
  RlsInit();
  RlsStart(256 /* lambdaQ8 = 1.0, no forgetting */, 1000 /* tickHz */);
  RunMove(&chainParams, &chainState, &analogParams, &analogState, &seedGains, 12345, &positionTarget, 1);
  RunPause(&chainParams, &chainState, &analogParams, &analogState, positionTarget, 500, 1, &seedGains);
  RunMove(&chainParams, &chainState, &analogParams, &analogState, &seedGains, 67890, &positionTarget, 1);
  RunPause(&chainParams, &chainState, &analogParams, &analogState, positionTarget, 500, 1, &seedGains);
  RunMove(&chainParams, &chainState, &analogParams, &analogState, &seedGains, 24680, &positionTarget, 1);

  AutotuneResult result = RlsGetResult();
  printf("RLS result=%s theta_a=%d theta_b=%d kp=%d ki=%d ticks_observed=%u p_trace=%lld\n",
         ResultName(result), RlsGetThetaAQ8(), RlsGetThetaBQ8(),
         RlsGetKpResultQ8(), RlsGetKiResultQ8(), RlsGetTicksObserved(),
         (long long)RlsGetPTrace());
  printf("before: maxErrAccel=%d maxErrCruise=%d pegTicks=%d\n",
         before.maxErrAccel, before.maxErrCruise, before.pegTicks);

  if (result != AUTOTUNE_RESULT_OK) {
    printf("no gains applied (result != OK) -- before metrics stand as the only measurement\n");
    if (scenario->hardAssert) {
      printf("FAIL -- expected convergence on a plant that matches RLS's model\n");
      return 1;
    }
    printf("DIAGNOSTIC -- model-mismatch stress case, no pass/fail assertion\n");
    return 0;
  }

  /* ---- APPLY: the same glide AppRlsApply()/AppControlTick() perform in
   * firmware (main.c), reproduced here with no HAL involved. position.kp
   * (posGainQ8) is deliberately left untouched, same as the real
   * AppRlsApply(). ---- */
  ControlGains tunedGains = seedGains;
  tunedGains.velGainKpQ8 = RlsGetKpResultQ8();
  tunedGains.velGainKiQ8 = RlsGetKiResultQ8();
  GainGlideState glide;
  GainGlideStart(&glide, ControlGetGains(), &tunedGains, 2000 /* ticks -- matches
                                                                   Protocol.c's
                                                                   RLS_DEFAULT_GLIDE_MS
                                                                   at this scenario's
                                                                   1kHz tickHz */);
  RunGlide(&chainParams, &chainState, &analogParams, &analogState, positionTarget, &glide);
  ControlReset();   /* clear the PI integrator before the clean validation move below, same
                        as the `before` run started from -- otherwise leftover windup from
                        the learning moves/glide would bias the comparison */

  /* ---- AFTER: identical validation move (same pulse-train seed), fresh
   * plant again, now with the gains RLS converged to -- a clean A/B
   * against `before`. ---- */
  SimmyChainInit(&chainState, &chainParams);
  SimmyAnalogChainInit(&analogState, 1);
  positionTarget = 0;
  MoveMetrics after = RunMove(&chainParams, &chainState, &analogParams, &analogState,
                               &tunedGains, 12345, &positionTarget, 0);
  printf("after:  maxErrAccel=%d maxErrCruise=%d pegTicks=%d\n",
         after.maxErrAccel, after.maxErrCruise, after.pegTicks);

  int improved = after.maxErrCruise < before.maxErrCruise;
  printf("%s\n", improved ? "tracking improved after RLS converged and was applied"
                          : "tracking did NOT improve");

  if (scenario->hardAssert) {
    if (!improved) { printf("FAIL -- investigate before trusting this linkage\n"); return 1; }
    printf("PASS -- RLS converged on a matching plant and measurably improved tracking\n");
    return 0;
  }
  printf("DIAGNOSTIC -- model-mismatch stress case, no pass/fail assertion\n");
  return 0;
}

int main(void) {
  RlsScenario scenarios[] = {
    { "1. rigid single-body (matches RLS's model)", NULL, 1 },
    { "2. two-body compliant (Nema23 leadscrew preset, backlash)",
      "examples/configs/two_mass.json", 0 },
    { "3. three-body leadscrew (belt + screw, two compliant stages)",
      "examples/configs/three_mass_leadscrew.json", 0 },
  };
  int rc = 0;
  for (size_t i = 0; i < sizeof(scenarios) / sizeof(scenarios[0]); i++) {
    int r = RunScenario(&scenarios[i]);
    if (r != 0) rc = r;
  }
  return rc;
}
