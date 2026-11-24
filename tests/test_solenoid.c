#include "simmy.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void near(double a, double b, double tolerance) { assert(fabs(a - b) < tolerance); }

/* Small shapeMeters relative to the working travel (sub-mm here) and a
 * 100x min/max inductance ratio: L rises sharply as the gap closes and
 * visibly flattens near the mechanical stop -- the iron-core/closed-flux
 * regime from the file header comment. */
static SimmyReluctanceParams IronCoreMotor(void) {
  return (SimmyReluctanceParams){
      .resistance = 2.0, .minInductance = .0005, .maxInductance = .05,
      .shapeMeters = .0005, .saturationSharpness = 2.0, .minGapMeters = .0002};
}

/* shapeMeters much larger than the same working travel and a modest 3x
 * ratio: L stays close to minInductance and barely more than linear --
 * the open-core/weakly-position-sensitive regime, same L(x) family. */
static SimmyReluctanceParams OpenCoreMotor(void) {
  return (SimmyReluctanceParams){
      .resistance = 2.0, .minInductance = .0005, .maxInductance = .0015,
      .shapeMeters = .02, .saturationSharpness = 1.0, .minGapMeters = .0002};
}

static void reluctance_params_valid(void) {
  SimmyReluctanceParams p = IronCoreMotor();
  assert(SimmyReluctanceParamsValid(&p));
  SimmyReluctanceParams bad = p; bad.resistance = -1; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.minInductance = 0; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.maxInductance = p.minInductance; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.maxInductance = p.minInductance * .5; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.shapeMeters = 0; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.saturationSharpness = 0; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.minGapMeters = 0; assert(!SimmyReluctanceParamsValid(&bad));
  bad = p; bad.resistance = NAN; assert(!SimmyReluctanceParamsValid(&bad));
}

/* One parameterized L(x) family represents BOTH the iron-core and
 * open-core regimes by choice of parameters alone (see the file header
 * comment) -- confirm the two sample motors above actually land in the
 * regimes their names claim: iron core should be far more position-
 * sensitive (big swing from near to far gap) than open core. */
static void one_family_two_regimes(void) {
  SimmyReluctanceParams iron = IronCoreMotor(), open = OpenCoreMotor();
  double nearGap = .0003, farGap = .003; /* 0.3mm and 3mm -- a plausible small-solenoid travel */
  double ironNear = SimmyReluctanceInductance(&iron, nearGap);
  double ironFar = SimmyReluctanceInductance(&iron, farGap);
  double openNear = SimmyReluctanceInductance(&open, nearGap);
  double openFar = SimmyReluctanceInductance(&open, farGap);
  assert(ironNear > ironFar && openNear > openFar); /* L always rises as gap closes */
  double ironRatio = ironNear / ironFar, openRatio = openNear / openFar;
  assert(ironRatio > 5 * openRatio); /* iron core is far more position-sensitive */
  assert(openRatio < 1.5);           /* open core is close to flat over this travel */

  /* Both still obey the same bounds regardless of regime. */
  for (int i = 0; i < 2; i++) {
    SimmyReluctanceParams *p = i ? &open : &iron;
    assert(SimmyReluctanceInductance(p, 1e6) > p->minInductance * .999); /* -> minInductance */
    assert(SimmyReluctanceInductance(p, p->minGapMeters) <= p->maxInductance + 1e-15); /* -> maxInductance, never exceeds it */
  }
  printf("Reluctance L(x): iron-core near/far ratio %.3g vs open-core %.3g -- one family, two regimes by parameter choice\n",
         ironRatio, openRatio);
}

/* Reluctance force must be ALWAYS ATTRACTIVE -- unsigned in the sense
 * that it never depends on current polarity, only current magnitude --
 * regardless of which sample motor or which gap. */
static void reluctance_force_always_attractive(void) {
  SimmyReluctanceParams motors[2] = {IronCoreMotor(), OpenCoreMotor()};
  for (int m = 0; m < 2; m++) {
    const SimmyReluctanceParams *p = &motors[m];
    for (double gap = p->minGapMeters; gap < .005; gap += .0004) {
      for (double voltage = -20; voltage <= 20; voltage += 5) {
        if (voltage == 0) continue;
        SimmyReluctanceState s; SimmyReluctanceInit(&s);
        SimmyReluctanceOutput o;
        for (int t = 0; t < 2000; t++) assert(SimmyReluctanceStep(&s, p, voltage, gap, 0, .0001, &o));
        assert(o.force <= 1e-9); /* never repulsive */
        if (fabs(s.current) > 1e-6) assert(o.force < -1e-12); /* strictly attractive once energized */
      }
    }
  }
  /* Same |voltage| magnitude, opposite polarity: force magnitude matches
   * (I^2 dependence), confirming polarity truly doesn't matter here. */
  SimmyReluctanceParams p = IronCoreMotor();
  SimmyReluctanceState sp, sn; SimmyReluctanceInit(&sp); SimmyReluctanceInit(&sn);
  SimmyReluctanceOutput op, on;
  for (int t = 0; t < 5000; t++) {
    assert(SimmyReluctanceStep(&sp, &p, 12, .001, 0, .0001, &op));
    assert(SimmyReluctanceStep(&sn, &p, -12, .001, 0, .0001, &on));
  }
  near(op.force, on.force, 1e-12);
  puts("Reluctance force: never repulsive for any current polarity/gap tested; |force| matches under reversed voltage.");
}

/* Flux-linkage conservation in a lossless, unexcited coil: with
 * resistance=0 and voltage=0, V = R*I + d(L(x)*I)/dt collapses to
 * d(psi)/dt = 0 -- psi = L(x)*I is EXACTLY conserved as the gap (and so
 * L) changes, however it changes, however fast. This is the sharpest
 * available check on the product-rule handling described in the file
 * header comment: a naive implementation that reuses a single L
 * evaluated at the CURRENT gap for both the stored and new terms of the
 * backward-Euler update would instead hold CURRENT (not flux) constant
 * across a gap change, which is wrong. */
static void flux_conservation_across_gap_changes(void) {
  SimmyReluctanceParams p = IronCoreMotor(); p.resistance = 0;
  SimmyReluctanceState s; SimmyReluctanceInit(&s);
  SimmyReluctanceOutput o;
  /* Charge the coil at the initial gap before removing the source and resistance. */
  SimmyReluctanceParams charge = p; charge.resistance = 2.0;
  assert(SimmyReluctanceStep(&s, &charge, 12, .002, 0, .01, &o));
  double psi = s.inductance * s.current;
  assert(fabs(s.current) > 1e-6);

  double gaps[] = {.0015, .0008, .0003, .0002, .001, .004};
  for (size_t i = 0; i < sizeof(gaps) / sizeof(gaps[0]); i++) {
    assert(SimmyReluctanceStep(&s, &p, 0, gaps[i], 0, .0005, &o)); /* R=0, V=0: psi must hold */
    double newPsi = s.inductance * s.current;
    near(newPsi, psi, fabs(psi) * 1e-9 + 1e-15);
    near(s.current * s.inductance, psi, fabs(psi) * 1e-9 + 1e-15);
  }
  puts("Reluctance product-rule fix: flux linkage L(x)*I holds exactly across gap changes with R=0, V=0.");
}

static double ReluctanceRlError(const SimmyReluctanceParams *p, double gap, double dt) {
  /* Gap held fixed: L(x) is then constant over the run, so this must
   * match the textbook scalar RL charging curve exactly, same closed
   * form SimmyDcStep's own convergence test checks. */
  SimmyReluctanceState s; SimmyReluctanceInit(&s);
  SimmyReluctanceOutput o;
  double l = SimmyReluctanceInductance(p, gap);
  int n = (int)round(.01 / dt);
  for (int i = 0; i < n; i++) assert(SimmyReluctanceStep(&s, p, 12, gap, 0, dt, &o));
  double exact = 12 / p->resistance * (1 - exp(-p->resistance / l * .01));
  return fabs(s.current - exact);
}
static void reluctance_convergence_fixed_gap(void) {
  SimmyReluctanceParams p = IronCoreMotor();
  double gap = .001;
  double e1 = ReluctanceRlError(&p, gap, .0001), e2 = ReluctanceRlError(&p, gap, .00005);
  assert(e2 < e1 * .55); /* backward Euler is first order */
  puts("Reluctance at fixed gap matches the closed-form RL charging curve (first-order convergence).");
}

static void reluctance_ledger_and_atomicity(void) {
  SimmyReluctanceParams p = IronCoreMotor();
  SimmyReluctanceState s; SimmyReluctanceInit(&s);
  SimmyReluctanceOutput o;
  double gap = .0015;
  for (int t = 0; t < 20000; t++) {
    gap = .0002 + .0013 * (1 + sin(t * .0007)); /* sweep the armature back and forth */
    double velocity = .0013 * .0007 / .00005 * cos(t * .0007);
    assert(SimmyReluctanceStep(&s, &p, 10 * sin(t * .0003), gap, velocity, .00005, &o));
  }
  double magneticEnergy = .5 * s.inductance * s.current * s.current;
  near(s.inputJ - s.mechanicalWorkJ - s.windingHeatJ - magneticEnergy, s.numericalLossJ, 1e-8);

  /* Caller-validates / fail-atomic convention, same as SimmyDcStep. */
  SimmyReluctanceState saved = s;
  SimmyReluctanceOutput sentinel; memset(&sentinel, 0x42, sizeof(sentinel)); o = sentinel;
  assert(!SimmyReluctanceStep(&s, &p, NAN, gap, 0, .0001, &o));
  assert(!memcmp(&s, &saved, sizeof(saved)) && !memcmp(&o, &sentinel, sizeof(o)));
  SimmyReluctanceParams badParams = p; badParams.maxInductance = 0;
  assert(!SimmyReluctanceStep(&s, &badParams, 10, gap, 0, .0001, &o));
  assert(!memcmp(&s, &saved, sizeof(saved)) && !memcmp(&o, &sentinel, sizeof(o)));
  puts("Reluctance energy ledger balances over a swept-gap trajectory; invalid input fails atomically.");
}

static void reluctance_chain_step_pulls_in(void) {
  SimmyChainParams chain = {0};
  chain.bodyCount = 1;
  chain.subSteps = 1;
  /* A comparatively heavy/damped mechanical load: the body chain's own
   * mechanical integrator is plain explicit Euler (see BodyMechanicalStep
   * in src/simmy.c), and this motor's force law gets very stiff near the
   * mechanical stop -- a light/underdamped load needs much finer
   * substepping than this test cares to exercise. Nothing to do with the
   * electrical model's own backward-Euler step, which stays implicit/
   * stable regardless. */
  chain.bodies[0].inertia = 2000;
  chain.bodies[0].friction = 100;
  SimmyChainState chainState;
  SimmyChainInit(&chainState, &chain);
  chainState.bodies[0].position = 3000; /* counts */

  SimmyLinearMount mount = {.metersPerCount = 1e-6, .forcePerNewton = 1, .tickSeconds = .0001};
  SimmyReluctanceParams motor = IronCoreMotor();
  SimmyReluctanceState motorState; SimmyReluctanceInit(&motorState);

  double startGap = chainState.bodies[0].position * mount.metersPerCount;
  for (int t = 0; t < 30000; t++) {
    assert(SimmyReluctanceChainStep(&chainState, &chain, &motorState, &motor, &mount, 6, mount.tickSeconds));
  }
  double endGap = chainState.bodies[0].position * mount.metersPerCount;
  assert(endGap < startGap);                     /* pulled toward smaller gap -- attraction, not repulsion */
  assert(endGap > motor.minGapMeters);            /* still well short of the mechanical stop */
  assert(chainState.bodies[0].velocity < 0);      /* still closing */
  puts("Reluctance ChainStep: energized coil pulls the armature toward smaller gap.");
}

/* ==================== Voice-coil / PM moving-magnet actuator ========== */

static SimmyVoiceCoilParams VoiceCoilMotor(void) {
  return (SimmyVoiceCoilParams){.resistance = 3.0, .inductance = .0006, .forceConstant = 1.2};
}

static void voicecoil_params_valid(void) {
  SimmyVoiceCoilParams p = VoiceCoilMotor();
  assert(SimmyVoiceCoilParamsValid(&p));
  SimmyVoiceCoilParams bad = p; bad.resistance = -1; assert(!SimmyVoiceCoilParamsValid(&bad));
  bad = p; bad.inductance = 0; assert(!SimmyVoiceCoilParamsValid(&bad));
  bad = p; bad.forceConstant = -1; assert(!SimmyVoiceCoilParamsValid(&bad));
}

static double VoiceCoilRlError(double dt) {
  SimmyVoiceCoilParams p = VoiceCoilMotor();
  SimmyVoiceCoilState s; SimmyVoiceCoilInit(&s);
  SimmyVoiceCoilOutput o;
  int n = (int)round(.01 / dt);
  for (int i = 0; i < n; i++) assert(SimmyVoiceCoilStep(&s, &p, 12, 0, dt, &o));
  double exact = 12 / p.resistance * (1 - exp(-p.resistance / p.inductance * .01));
  return fabs(s.current - exact);
}

/* The defining difference from the reluctance family: reversing current
 * reverses force direction -- this is what makes push AND pull possible,
 * which a reluctance solenoid structurally cannot do (see the file
 * header comment and reluctance_force_always_attractive() above). */
static void voicecoil_force_is_signed(void) {
  SimmyVoiceCoilParams p = VoiceCoilMotor();
  SimmyVoiceCoilState s; SimmyVoiceCoilInit(&s);
  SimmyVoiceCoilOutput o;
  for (int t = 0; t < 5000; t++) assert(SimmyVoiceCoilStep(&s, &p, 12, 0, .0001, &o));
  double steadyCurrent = 12 / p.resistance;
  near(s.current, steadyCurrent, 1e-6);
  assert(o.force > 0);
  near(o.force, p.forceConstant * steadyCurrent, 1e-12);

  SimmyVoiceCoilInit(&s);
  for (int t = 0; t < 5000; t++) assert(SimmyVoiceCoilStep(&s, &p, -12, 0, .0001, &o));
  assert(o.force < 0);
  near(o.force, -p.forceConstant * steadyCurrent, 1e-12);
}

static void voicecoil_convergence_ledger_and_atomicity(void) {
  double e1 = VoiceCoilRlError(.0001), e2 = VoiceCoilRlError(.00005);
  assert(e2 < e1 * .55);

  SimmyVoiceCoilParams p = VoiceCoilMotor();
  SimmyVoiceCoilState s; SimmyVoiceCoilInit(&s);
  SimmyVoiceCoilOutput o;
  double speed = 4, voltage = 20;
  for (int i = 0; i < 200000; i++) assert(SimmyVoiceCoilStep(&s, &p, voltage, speed, .0001, &o));
  double steadyCurrent = (voltage - p.forceConstant * speed) / p.resistance;
  near(s.current, steadyCurrent, 1e-4);
  near(s.inputJ - s.mechanicalWorkJ - s.windingHeatJ - .5 * p.inductance * s.current * s.current,
       s.numericalLossJ, 2e-6);

  SimmyVoiceCoilState saved = s;
  SimmyVoiceCoilOutput sentinel; memset(&sentinel, 0x42, sizeof(sentinel)); o = sentinel;
  assert(!SimmyVoiceCoilStep(&s, &p, NAN, speed, .0001, &o));
  assert(!memcmp(&s, &saved, sizeof(saved)) && !memcmp(&o, &sentinel, sizeof(o)));
  SimmyVoiceCoilParams bad = p; bad.inductance = 0;
  assert(!SimmyVoiceCoilStep(&s, &bad, voltage, speed, .0001, &o));
  assert(!memcmp(&s, &saved, sizeof(saved)) && !memcmp(&o, &sentinel, sizeof(o)));
  puts("Voice coil: RL step response matches closed form; energy ledger balances; invalid input fails atomically.");
}

static void voicecoil_chain_step_pushes_and_pulls(void) {
  SimmyChainParams chain = {0};
  chain.bodyCount = 1;
  chain.subSteps = 1;
  /* Same heavier/damped load as reluctance_chain_step_pulls_in(), for the
   * same reason: the mechanical side is plain explicit Euler, so a very
   * light/underdamped load driven this hard needs finer substepping than
   * this test exercises. */
  chain.bodies[0].inertia = 2000;
  chain.bodies[0].friction = 100;
  SimmyChainState chainState;
  SimmyChainInit(&chainState, &chain);

  SimmyLinearMount mount = {.metersPerCount = 1e-6, .forcePerNewton = 1, .tickSeconds = .0001};
  SimmyVoiceCoilParams motor = VoiceCoilMotor();
  SimmyVoiceCoilState motorState; SimmyVoiceCoilInit(&motorState);

  for (int t = 0; t < 20000; t++)
    assert(SimmyVoiceCoilChainStep(&chainState, &chain, &motorState, &motor, &mount, 12, mount.tickSeconds));
  assert(chainState.bodies[0].velocity > 0); /* pushed one way */

  SimmyChainInit(&chainState, &chain);
  SimmyVoiceCoilInit(&motorState);
  for (int t = 0; t < 20000; t++)
    assert(SimmyVoiceCoilChainStep(&chainState, &chain, &motorState, &motor, &mount, -12, mount.tickSeconds));
  assert(chainState.bodies[0].velocity < 0); /* and pulled the other way -- unlike reluctance */
  puts("Voice-coil ChainStep: reversing voltage reverses the resulting motion direction.");
}

/* Exercise both JSON loaders, including missing and invalid parameters. */
static void json_parameters(void) {
  const char *dir = getenv("TMPDIR");
  if (!dir || !*dir) dir = "build";
  char path[1024], err[256];
  snprintf(path, sizeof(path), "%s/simmy-test-solenoid.json", dir);
  const char *cases[] = {
      "{\"reluctanceMotor\":{\"resistance\":2,\"minInductance\":0.0005,"
      "\"maxInductance\":0.05,\"shapeMeters\":0.0005,"
      "\"saturationSharpness\":2,\"minGapMeters\":0.0002},"
      "\"voiceCoilMotor\":{\"resistance\":3,\"inductance\":0.0006,\"forceConstant\":1.2}}",
      "{}",
      "{\"reluctanceMotor\":{\"minInductance\":0.01,\"maxInductance\":0.001,"
      "\"shapeMeters\":0.001,\"saturationSharpness\":1,\"minGapMeters\":0.0001},"
      "\"voiceCoilMotor\":{\"inductance\":0.001,\"forceConstant\":-1}}",
      "{invalid json"
  };
  for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
    FILE *file = fopen(path, "wb");
    assert(file);
    assert(fputs(cases[i], file) >= 0);
    assert(fclose(file) == 0);
    SimmyReluctanceParams reluctance;
    SimmyVoiceCoilParams voicecoil;
    assert((SimmyReluctanceParamsFromJsonFile(path, &reluctance, err, sizeof(err)) == 0) == (i == 0));
    assert((SimmyVoiceCoilParamsFromJsonFile(path, &voicecoil, err, sizeof(err)) == 0) == (i == 0));
    if (i == 0) {
      assert(SimmyReluctanceParamsValid(&reluctance));
      assert(SimmyVoiceCoilParamsValid(&voicecoil));
      near(reluctance.maxInductance, .05, 1e-12);
      near(voicecoil.forceConstant, 1.2, 1e-12);
    } else {
      assert(err[0]);
    }
  }
  assert(remove(path) == 0);
}

int main(void) {
  json_parameters();
  reluctance_params_valid();
  one_family_two_regimes();
  reluctance_force_always_attractive();
  flux_conservation_across_gap_changes();
  reluctance_convergence_fixed_gap();
  reluctance_ledger_and_atomicity();
  reluctance_chain_step_pulls_in();
  voicecoil_params_valid();
  voicecoil_force_is_signed();
  voicecoil_convergence_ledger_and_atomicity();
  voicecoil_chain_step_pushes_and_pulls();
  puts("Reluctance + voice-coil linear actuator tests passed.");
  return 0;
}
