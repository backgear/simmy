/*
 * Plain assert()-based test suite -- no Unity dependency, since that's a
 * PlatformIO/analog-servo fixture, not appropriate for a standalone repo.
 * Verifies simmy's own physics/API correctness in isolation (no external
 * controller linked in here -- see examples/analog_servo_integration/
 * for the real-Control.c regression tie-back). Run: make test
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "simmy.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* A deliberately tiny demo controller, for tests that need a closed loop
 * -- an ordinary SimmyControllerFn implementation, same convention a real
 * one would use. Needs the velocity (damping) term: a frictionless mass
 * under pure position-proportional control is marginally stable (a bare
 * mass-spring system, no damping) and oscillates forever rather than
 * converging -- not a simmy quirk, ordinary control theory for a double
 * integrator. */
static int32_t DemoPdController(void *userData, int32_t targetPosition, int32_t actualPosition,
                                 int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  (void)dtCycles;
  int64_t posError = (int64_t)targetPosition - actualPosition;
  int64_t command = posError - 20 * (int64_t)actualVelocity;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  return (int32_t)command;
}

/* ---- single-body (rigid) chain --------------------------------------- */

static void test_single_body_constant_force_matches_closed_form(void) {
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 1;
  params.bodies[0].inertia = 16.0;
  params.feedbackBody = 0;

  SimmyChainState state;
  SimmyChainInit(&state, &params);

  /* Frictionless, cogging-free single mass: velocity after N ticks of a
   * constant force F is exactly N*F/inertia (a closed-form check, not
   * just "stays bounded"). */
  double force = 160.0;
  int ticks = 100;
  for (int i = 0; i < ticks; i++) SimmyChainStep(&state, &params, force);

  double expectedVelocity = ticks * force / params.bodies[0].inertia;
  int32_t actualVelocity = SimmyChainFeedbackVelocityCounts(&state, &params);
  assert(Abs32((int32_t)expectedVelocity - actualVelocity) <= 1);   /* rounding to whole counts */
}

static void test_single_body_stiction_holds_below_breakaway(void) {
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 1;
  params.bodies[0].inertia = 16.0;
  params.bodies[0].stictionLimit = 100.0;

  SimmyChainState state;
  SimmyChainInit(&state, &params);

  for (int i = 0; i < 500; i++) SimmyChainStep(&state, &params, 50.0);   /* below breakaway */
  assert(SimmyChainFeedbackPositionCounts(&state, &params) == 0);
  assert(SimmyChainFeedbackVelocityCounts(&state, &params) == 0);

  for (int i = 0; i < 50; i++) SimmyChainStep(&state, &params, 150.0);   /* above breakaway */
  assert(SimmyChainFeedbackVelocityCounts(&state, &params) > 0);        /* actually moved */
}

static void test_single_body_stiction_does_not_chatter(void) {
  /* The classic naive-implementation bug: a post-hoc velocity clamp makes
   * velocity oscillate around 0 every tick instead of genuinely sticking.
   * Confirms velocity stays EXACTLY 0 (not just small) while stuck. */
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 1;
  params.bodies[0].inertia = 16.0;
  params.bodies[0].stictionLimit = 100.0;

  SimmyChainState state;
  SimmyChainInit(&state, &params);
  for (int i = 0; i < 200; i++) {
    SimmyChainStep(&state, &params, 90.0);
    assert(state.bodies[0].velocity == 0.0);
  }
}

static void test_friction_bump_is_localized_and_measurable(void) {
  SimmyChainParams noBump, withBump;
  memset(&noBump, 0, sizeof(noBump));
  noBump.bodyCount = 1;
  noBump.bodies[0].inertia = 16.0;
  noBump.bodies[0].friction = 0.05;

  withBump = noBump;
  withBump.bodies[0].frictionBumpCenter = 500.0;
  withBump.bodies[0].frictionBumpHalfWidth = 20.0;
  withBump.bodies[0].frictionBumpExtra = 40.0;

  SimmyChainState s1, s2;
  SimmyChainInit(&s1, &noBump);
  SimmyChainInit(&s2, &withBump);

  int32_t maxDiff = 0;
  for (int i = 0; i < 2000; i++) {
    SimmyChainStep(&s1, &noBump, 20.0);
    SimmyChainStep(&s2, &withBump, 20.0);
    int32_t diff = Abs32(SimmyChainFeedbackPositionCounts(&s1, &noBump) -
                          SimmyChainFeedbackPositionCounts(&s2, &withBump));
    if (diff > maxDiff) maxDiff = diff;
  }
  assert(maxDiff > 0);   /* the bump measurably changed the trajectory */
}

static void test_ripple_is_periodic_in_position_not_time(void) {
  /* A body held at a FIXED position (huge inertia, tiny force) should see
   * the same ripple contribution every time it returns to the same
   * position -- confirms the ripple is a function of position, not tick
   * count. Checked indirectly: two runs starting at different initial
   * positions but separated by exactly one ripple period apart should
   * produce identical velocity trajectories under the same force. */
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 1;
  params.bodies[0].inertia = 16.0;
  params.bodies[0].rippleAmplitude = 30.0;
  params.bodies[0].ripplePeriodCounts = 10.0;

  SimmyChainState a, b;
  SimmyChainInit(&a, &params);
  SimmyChainInit(&b, &params);
  b.bodies[0].position = 10.0;   /* exactly one ripple period ahead */

  for (int i = 0; i < 50; i++) {
    SimmyChainStep(&a, &params, 5.0);
    SimmyChainStep(&b, &params, 5.0);
    assert(fabs(a.bodies[0].velocity - b.bodies[0].velocity) < 1e-9);
  }
}

/* ---- two-body coupled chain ------------------------------------------- */

static void test_two_body_free_oscillation_decays(void) {
  /* Numerical-stability sanity check for the sub-stepped coupling: a
   * displaced, undamped-ish spring+damper system released with no
   * external force should decay, not grow (the classic plain-Euler
   * instability risk this sub-stepping exists to avoid). */
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 2;
  params.bodies[0].inertia = 64.0;
  params.bodies[1].inertia = 8.0;
  params.couplings[0].springK = 3.16;
  params.couplings[0].springD = 1.0;
  params.subSteps = 20;

  SimmyChainState state;
  SimmyChainInit(&state, &params);
  state.bodies[1].position = 100.0;

  double maxRelFirst = 0, maxRelLast = 0;
  int ticks = 2000;
  for (int i = 0; i < ticks; i++) {
    SimmyChainStep(&state, &params, 0.0);
    double rel = fabs(state.bodies[0].position - state.bodies[1].position);
    if (i < 100 && rel > maxRelFirst) maxRelFirst = rel;
    if (i >= ticks - 100 && rel > maxRelLast) maxRelLast = rel;
  }
  assert(maxRelLast < maxRelFirst);
}

/* Drives body 0 forward (loading the coupling onto the "forward" side of
 * the backlash gap, confirmed by checking relative displacement exceeds
 * the gap) then reverses, and confirms the relative displacement
 * actually passes back THROUGH the dead zone (|rel| < backlashGap, i.e.
 * zero transmitted force) at some point -- the direct, physical
 * signature of a real backlash gap engaging on reversal. (A naive
 * "does the load's velocity respond more slowly" proxy turned out not
 * to be reliable here: backlash delays the START of the load's response
 * but then re-engages more abruptly, so total response time isn't
 * simply slower -- the dead-zone crossing itself is the real thing to
 * check for.) */
static void test_two_body_backlash_creates_dead_zone_on_reversal(void) {
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 2;
  params.bodies[0].inertia = 16.0;
  params.bodies[1].inertia = 16.0;
  params.couplings[0].springK = 5.0;
  params.couplings[0].springD = 0.5;
  params.couplings[0].backlashGap = 10.0;
  params.subSteps = 20;

  SimmyChainState state;
  SimmyChainInit(&state, &params);
  for (int i = 0; i < 20; i++) SimmyChainStep(&state, &params, 50.0);   /* load the forward side */
  double relBeforeReversal = state.bodies[0].position - state.bodies[1].position;
  assert(fabs(relBeforeReversal) > params.couplings[0].backlashGap);

  int sawDeadZone = 0;
  for (int i = 0; i < 100; i++) {
    SimmyChainStep(&state, &params, -50.0);   /* reverse */
    double rel = state.bodies[0].position - state.bodies[1].position;
    if (fabs(rel) < params.couplings[0].backlashGap) sawDeadZone = 1;
  }
  assert(sawDeadZone);
}

/* ---- analog signal chain ------------------------------------------------ */

static void test_analog_chain_composes_stages_in_order(void) {
  SimmyAnalogChainParams params;
  memset(&params, 0, sizeof(params));
  params.stageCount = 3;
  params.stages[0] = (SimmyAnalogStageParams){.type = SIMMY_STAGE_GAIN, .param0 = 2.0};
  params.stages[1] = (SimmyAnalogStageParams){.type = SIMMY_STAGE_SATURATE, .param0 = -100, .param1 = 100};
  params.stages[2] = (SimmyAnalogStageParams){.type = SIMMY_STAGE_QUANTIZE, .param0 = 10.0};

  SimmyAnalogChainState state;
  SimmyAnalogChainInit(&state, 1);

  /* 23 * 2.0 = 46 -> saturate to [-100,100], unchanged -> quantize to
   * nearest multiple of 10 -> 50. */
  assert(fabs(SimmyAnalogChainStep(&state, &params, 23.0) - 50.0) < 1e-9);
}

static void test_analog_chain_lag_responds_gradually(void) {
  SimmyAnalogChainParams params;
  memset(&params, 0, sizeof(params));
  params.stageCount = 1;
  params.stages[0] = (SimmyAnalogStageParams){.type = SIMMY_STAGE_LAG, .param0 = 0.1};

  SimmyAnalogChainState state;
  SimmyAnalogChainInit(&state, 1);

  double first = SimmyAnalogChainStep(&state, &params, 100.0);
  assert(first > 0.0 && first < 100.0);   /* not instant */
  double last = 0;
  for (int i = 0; i < 200; i++) last = SimmyAnalogChainStep(&state, &params, 100.0);
  assert(fabs(last - 100.0) < 1e-6);      /* settles to the input */
}

/* ---- bidirectional pulse source ----------------------------------------- */

static void test_pulse_train_negative_rate_is_symmetric_to_positive(void) {
  SimmyPulseTrainState posState, negState;
  SimmyPulseTrainInit(&posState, 42);
  SimmyPulseTrainInit(&negState, 42);

  int32_t posSum = 0, negSum = 0;
  int ticks = 1000;
  for (int i = 0; i < ticks; i++) {
    posSum += SimmyPulseTrainTickDelta(&posState, 2500.0, 1000.0, 0.0);
    negSum += SimmyPulseTrainTickDelta(&negState, -2500.0, 1000.0, 0.0);
  }
  assert(posSum == -negSum);   /* exact symmetry for a clean (no-jitter) rate */
}

static void test_pulse_train_reversal_mid_run_is_clean(void) {
  SimmyPulseTrainState state;
  SimmyPulseTrainInit(&state, 7);
  int32_t total = 0;
  for (int i = 0; i < 500; i++) total += SimmyPulseTrainTickDelta(&state, 2500.0, 1000.0, 0.2);
  int32_t afterForward = total;
  for (int i = 0; i < 500; i++) total += SimmyPulseTrainTickDelta(&state, -2500.0, 1000.0, 0.2);
  /* Should return close to where it started -- not exactly 0 (jitter), but
   * nowhere near afterForward (confirms the reversal actually reversed). */
  assert(Abs32(total) < afterForward / 4);
}

static void test_pulse_train_jitter_never_flips_sign(void) {
  SimmyPulseTrainState state;
  SimmyPulseTrainInit(&state, 99);
  for (int i = 0; i < 5000; i++) {
    int32_t delta = SimmyPulseTrainTickDelta(&state, 50.0, 1000.0, 1.0);   /* max jitter, slow rate */
    assert(delta >= 0);   /* a positive rate must never produce a negative tick delta */
  }
}

/* ---- JSON config loading ------------------------------------------------- */

static void WriteFile(const char *path, const char *contents) {
  FILE *f = fopen(path, "wb");
  assert(f);
  fputs(contents, f);
  fclose(f);
}

static void test_json_loads_two_body_config(void) {
  const char *json =
      "{\n"
      "  \"bodies\": [\n"
      "    {\"inertia\": 64, \"friction\": 0.02, \"rippleAmplitude\": 60, \"ripplePeriodCounts\": 20},\n"
      "    {\"inertia\": 8, \"friction\": 0.01, \"stictionLimit\": 80, \"coulombLevel\": 50}\n"
      "  ],\n"
      "  \"couplings\": [ {\"springK\": 3.16, \"springD\": 1.0, \"backlashGap\": 0.5} ],\n"
      "  \"feedbackBody\": 0,\n"
      "  \"subSteps\": 20,\n"
      "  \"analogChain\": [ {\"type\": \"lag\", \"param0\": 0.6} ]\n"
      "}\n";
  WriteFile("/tmp/simmy_test_config.json", json);

  SimmyChainParams params;
  char err[256] = {0};
  int rc = SimmyChainParamsFromJsonFile("/tmp/simmy_test_config.json", &params, err, sizeof(err));
  assert(rc == 0);
  assert(params.bodyCount == 2);
  assert(params.bodies[0].inertia == 64.0);
  assert(params.bodies[1].stictionLimit == 80.0);
  assert(params.couplings[0].springK == 3.16);
  assert(params.subSteps == 20);

  SimmyAnalogChainParams analogParams;
  rc = SimmyAnalogChainParamsFromJsonFile("/tmp/simmy_test_config.json", &analogParams, err, sizeof(err));
  assert(rc == 0);
  assert(analogParams.stageCount == 1);
  assert(analogParams.stages[0].type == SIMMY_STAGE_LAG);
  assert(analogParams.stages[0].param0 == 0.6);
}

static void test_json_reports_missing_file(void) {
  SimmyChainParams params;
  char err[256] = {0};
  int rc = SimmyChainParamsFromJsonFile("/tmp/this_file_does_not_exist.json", &params, err, sizeof(err));
  assert(rc != 0);
  assert(err[0] != '\0');
}

static void test_json_requires_positive_inertia(void) {
  WriteFile("/tmp/simmy_test_bad_inertia.json", "{\"bodies\": [{\"friction\": 0.1}]}");
  SimmyChainParams params;
  char err[256] = {0};
  int rc = SimmyChainParamsFromJsonFile("/tmp/simmy_test_bad_inertia.json", &params, err, sizeof(err));
  assert(rc != 0);
}

/* ---- closed-loop sanity with the demo controller ------------------------ */

static void test_closed_loop_with_demo_controller_converges(void) {
  SimmyChainParams params;
  memset(&params, 0, sizeof(params));
  params.bodyCount = 1;
  params.bodies[0].inertia = 16.0;

  SimmyChainState state;
  SimmyChainInit(&state, &params);

  SimmyControllerFn controller = DemoPdController;
  int32_t target = 5000;
  int32_t lastErr = 0;
  for (int i = 0; i < 3000; i++) {
    int32_t actualPos = SimmyChainFeedbackPositionCounts(&state, &params);
    int32_t actualVel = SimmyChainFeedbackVelocityCounts(&state, &params);
    int32_t torque = controller(NULL, target, actualPos, actualVel, 32000);
    lastErr = target - actualPos;
    SimmyChainStep(&state, &params, (double)torque);
  }
  assert(Abs32(lastErr) < 50);
}

int main(void) {
  test_single_body_constant_force_matches_closed_form();
  test_single_body_stiction_holds_below_breakaway();
  test_single_body_stiction_does_not_chatter();
  test_friction_bump_is_localized_and_measurable();
  test_ripple_is_periodic_in_position_not_time();
  test_two_body_free_oscillation_decays();
  test_two_body_backlash_creates_dead_zone_on_reversal();
  test_analog_chain_composes_stages_in_order();
  test_analog_chain_lag_responds_gradually();
  test_pulse_train_negative_rate_is_symmetric_to_positive();
  test_pulse_train_reversal_mid_run_is_clean();
  test_pulse_train_jitter_never_flips_sign();
  test_json_loads_two_body_config();
  test_json_reports_missing_file();
  test_json_requires_positive_inertia();
  test_closed_loop_with_demo_controller_converges();
  printf("All simmy tests passed.\n");
  return 0;
}
