/*
 * Same "wrap the real RLS/GainGlide/Control firmware modules" harness as
 * examples/rls_integration/main.c, but against the REAL frozen empirical
 * SM232AE/TQ10 axis model (axis_command_step(), models/axis/
 * sm232ae_tq10_command_v2.json) instead of a simmy chain. Sim-only: this
 * checks whether RLS's online self-tuner converges to useful gains on a
 * frozen nonlinear plant. It never touches real hardware and does not
 * establish hardware agreement. See this directory's README.md for the
 * demo interface and the older sweeps' measurement limitations.
 *
 * Units: the model's own state is mechanical radians/radians-per-second
 * and a command normalized to code/32767, not measured current or torque;
 * Control.c works in encoder counts/counts-per-tick at a fixed 1kHz.
 * Converted via the model's own encoder_counts_per_rev (4096), loaded
 * from the same frozen JSON file as every other coefficient here --
 * nothing in this file is a second, independently-typed copy of that
 * model.
 *
 * No hard pass/fail assertion: unlike rls_integration's rigid scenario
 * (deliberately built to match RLS's single-pole v[n+1]=a*v[n]+b*u[n]
 * model), this plant has asymmetric gain/loss/viscous terms, a command
 * filter, a deadband, and stiction -- closer to rls_integration's
 * scenario 2/3 (model mismatch), where RLS's own clamp/BAD_MEASUREMENT
 * safety net is the thing actually being exercised. Both outcomes
 * (converges and improves, or correctly refuses) are reported honestly.
 *
 * Only builds/runs via `make rls-axis-command-v2` (requires the
 * ANALOG_SERVO_RLS_DIR sibling repo; see the Makefile). Run from the
 * simmy repo root.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "Control.h"
#include "RLS.h"
#include "GainGlide.h"
#include "simmy.h"
#include "simmy_json.h"
#include "../axis_command_step.h"

#define MODEL_PATH "models/axis/sm232ae_tq10_command_v2.json"
#define CONTROL_TICK_HZ 1000
#define PLANT_SUBSTEPS 2 /* model's own native step is 0.5ms -- 2x this harness's 1ms control tick */
#define PLANT_DT (1.0 / CONTROL_TICK_HZ / PLANT_SUBSTEPS)
#define COMMAND_FULL_SCALE 32767.0

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* Reads the real frozen model file -- the same one
 * examples/axis_pc_tune.py and axis_model_fit_v2.py both use -- via
 * simmy's own small JSON
 * parser (src/simmy_json.h), rather than copying its coefficients as a
 * second, independently-typed literal array that could drift out of sync
 * with the frozen file. */
static int LoadModel(double p[8], double *countsPerRad) {
  FILE *f = fopen(MODEL_PATH, "rb");
  if (!f) {
    fprintf(stderr, "cannot open %s\n", MODEL_PATH);
    return -1;
  }
  fseek(f, 0, SEEK_END);
  long len = ftell(f);
  fseek(f, 0, SEEK_SET);
  char *buf = malloc((size_t)len + 1);
  if (fread(buf, 1, (size_t)len, f) != (size_t)len) {
    fprintf(stderr, "short read on %s\n", MODEL_PATH);
    fclose(f);
    free(buf);
    return -1;
  }
  buf[len] = '\0';
  fclose(f);

  char err[256];
  SimmyJsonValue *root = SimmyJsonParse(buf, err, sizeof(err));
  free(buf);
  if (!root) {
    fprintf(stderr, "parse error in %s: %s\n", MODEL_PATH, err);
    return -1;
  }

  const SimmyJsonValue *params = SimmyJsonObjectGet(root, "parameters");
  static const char *names[8] = {
      "positive_gain_rad_s2_per_command", "negative_gain_rad_s2_per_command",
      "positive_constant_loss_rad_s2",    "negative_constant_loss_rad_s2",
      "positive_viscous_s_inv",           "negative_viscous_s_inv",
      "command_filter_tau_s",             "command_deadband_fraction",
  };
  for (int i = 0; i < 8; i++) {
    p[i] = SimmyJsonNumber(SimmyJsonObjectGet(params, names[i]), 0.0);
  }
  double countsPerRev = SimmyJsonNumber(SimmyJsonObjectGet(root, "encoder_counts_per_rev"), 4096.0);
  *countsPerRad = countsPerRev / (2.0 * M_PI);
  SimmyJsonFree(root);
  return 0;
}

typedef struct {
  int32_t maxErrAccel, maxErrCruise, pegTicks;
} MoveMetrics;

static int32_t PlantPositionCounts(const AxisCommandState *plant, double countsPerRad) {
  return (int32_t)lround(plant->x * countsPerRad);
}
/* Rounded truth, not encoder feedback. Older RLS sweep paths still use this
 * model-only observation. The README demo uses encoder differences instead;
 * retain truth only as a diagnostic there. */
static int32_t PlantVelocityCountsPerTick(const AxisCommandState *plant, double countsPerRad) {
  return (int32_t)lround(plant->v * countsPerRad / CONTROL_TICK_HZ);
}
static void StepPlant(AxisCommandState *plant, int32_t torque, const double *p) {
  double raw = (double)torque / COMMAND_FULL_SCALE;
  for (int s = 0; s < PLANT_SUBSTEPS; s++) axis_command_step(plant, raw, PLANT_DT, p);
}

#include "demo_tick.h"

/* Same ramped/jittered move shape as rls_integration's own RunMove --
 * proven RLS excitation, reused here against this plant's real
 * velocityClamp/torqueClamp instead of simmy's. */
static MoveMetrics RunMove(AxisCommandState *plant, const double *p, double countsPerRad,
                           const ControlGains *gains, uint32_t pulseSeed, int32_t *positionTarget,
                           int feedRls) {
  SimmyPulseTrainState pulseState;
  SimmyPulseTrainInit(&pulseState, pulseSeed);

  double targetRate = 60000.0, rampSeconds = 1.0, jitterFraction = 0.3; /* ~60 counts/tick cruise,
                                                                            under the 90 cap above */
  int accelTicks = 1000, cruiseTicks = 1000, ticks = accelTicks + cruiseTicks;
  double accelRatePerSec2 = targetRate / rampSeconds;

  MoveMetrics m = {0, 0, 0};
  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / 1000.0) : targetRate;
    int32_t delta = SimmyPulseTrainTickDelta(&pulseState, rate, (double)CONTROL_TICK_HZ, jitterFraction);
    *positionTarget += delta;

    int32_t position = PlantPositionCounts(plant, countsPerRad);
    int32_t velocity = PlantVelocityCountsPerTick(plant, countsPerRad);
    int32_t torque = ControlTick(*positionTarget, position, velocity, 32000);
    if (feedRls) {
      torque += RlsGetDitherTorque();
      if (torque > gains->torqueClampMax) torque = gains->torqueClampMax;
      if (torque < gains->torqueClampMin) torque = gains->torqueClampMin;
      RlsTick(torque, velocity);
    }
    StepPlant(plant, torque, p);

    int32_t err = Abs32(*positionTarget - position);
    if (i < accelTicks) {
      if (err > m.maxErrAccel) m.maxErrAccel = err;
    } else {
      if (err > m.maxErrCruise) m.maxErrCruise = err;
    }
    if (Abs32(torque) >= gains->torqueClampMax - 1) m.pegTicks++;
  }
  return m;
}

static void RunPause(AxisCommandState *plant, const double *p, double countsPerRad,
                      const ControlGains *gains, int32_t positionTarget, uint32_t ticks, int feedRls) {
  for (uint32_t i = 0; i < ticks; i++) {
    int32_t position = PlantPositionCounts(plant, countsPerRad);
    int32_t velocity = PlantVelocityCountsPerTick(plant, countsPerRad);
    int32_t torque = ControlTick(positionTarget, position, velocity, 32000);
    if (feedRls) {
      torque += RlsGetDitherTorque();
      if (torque > gains->torqueClampMax) torque = gains->torqueClampMax;
      if (torque < gains->torqueClampMin) torque = gains->torqueClampMin;
      RlsTick(torque, velocity);
    }
    StepPlant(plant, torque, p);
  }
}

static void RunGlide(AxisCommandState *plant, const double *p, double countsPerRad,
                      int32_t positionTarget, GainGlideState *glide) {
  while (GainGlideActive(glide)) {
    ControlGains glided;
    if (GainGlideTick(glide, &glided)) ControlSetGains(&glided);
    int32_t position = PlantPositionCounts(plant, countsPerRad);
    int32_t velocity = PlantVelocityCountsPerTick(plant, countsPerRad);
    int32_t torque = ControlTick(positionTarget, position, velocity, 32000);
    StepPlant(plant, torque, p);
  }
}

static const char *ResultName(AutotuneResult r) {
  switch (r) {
    case AUTOTUNE_RESULT_OK:
      return "OK";
    case AUTOTUNE_RESULT_BAD_MEASUREMENT:
      return "BAD_MEASUREMENT";
    default:
      return "NONE";
  }
}

typedef struct {
  int converged, improved, beatsKnownGood, trials;
} SweepSummary;

/* One knob per RLS.c "generous, bench-TBD placeholder" default (see RLS.h's
 * sweep-only setters) plus the sweep's own RNG seed, bundled so RunSweep's
 * signature doesn't grow a new positional argument per knob swept. Defaults
 * here mirror RLS.c's shipped #define values exactly -- gainMaxQ16's 65536
 * is RLS_SCALE (Q16.16 "1.0"), not independently chosen. */
typedef struct {
  int32_t aggressiveness, lambdaQ8, glideMs;
  int32_t bMaxAtRefQ8, uMin, vMin, ditherPctQ8;
  int64_t gainMaxQ16;
  uint32_t rngSeed;
} SweepParams;

static const SweepParams kDefaultSweepParams = {
    .aggressiveness = 5, .lambdaQ8 = 256, .glideMs = 2000,
    .bMaxAtRefQ8 = 64, .uMin = 50, .vMin = 2, .ditherPctQ8 = 13,
    .gainMaxQ16 = 2000LL * 65536LL, .rngSeed = 0x9e3779b9u,
};

/* Runs the same 100-point randomized velocity-gain sweep at the given
 * SweepParams. perTrialCsv, if non-NULL, gets one row per trial (including
 * rejected ones) -- used only for the one configuration this repo settled
 * on, to plot the actual before/after distribution; every configuration
 * still contributes to the summary counts either way. */
static SweepSummary RunSweep(const double *p, double countsPerRad, int32_t knownGoodCruise,
                              SweepParams sp, FILE *perTrialCsv) {
  SweepSummary s = {0, 0, 0, 100};
  uint32_t rng = sp.rngSeed; /* deterministic sweep, not a claim about real randomness
                                  (see docs/scope.md's own determinism note: this is the caller's own
                                  code picking starting points, not simmy's physics calling rand()) */
  for (int trial = 0; trial < s.trials; trial++) {
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    int32_t velGainKpQ8 = 2000 + (int32_t)(rng % 198001); /* actual Kp ~7.8 .. 781 */
    rng ^= rng << 13;
    rng ^= rng >> 17;
    rng ^= rng << 5;
    int32_t velGainKiQ8 = (int32_t)(rng % 20001); /* actual Ki 0 .. ~78 */

    /* posGainQ8/velocityClamp/torqueClamp fixed at the real search's own
     * selected position gain and this harness's extrapolated-speed clamps
     * (see the file comment) -- only the velocity-loop seed varies, since
     * that's the one RLS itself tunes. */
    ControlGains seedGains = {
        .posGainQ8 = 16,
        .velGainKpQ8 = velGainKpQ8,
        .velGainKiQ8 = velGainKiQ8,
        .velocityClamp = 90,
        .torqueClampMin = -3276,
        .torqueClampMax = 3276,
    };

    AxisCommandState plant;
    int32_t positionTarget;

    ControlInit(&seedGains);
    memset(&plant, 0, sizeof(plant));
    positionTarget = 0;
    MoveMetrics before = RunMove(&plant, p, countsPerRad, &seedGains, 12345, &positionTarget, 0);

    ControlInit(&seedGains);
    memset(&plant, 0, sizeof(plant));
    positionTarget = 0;
    RlsInit();
    RlsSetBMaxAtRefQ8(sp.bMaxAtRefQ8);
    RlsSetGainMaxQ16(sp.gainMaxQ16);
    RlsSetDeadzone(sp.uMin, sp.vMin);
    RlsSetDitherPctQ8(sp.ditherPctQ8);
    RlsStart(sp.lambdaQ8, CONTROL_TICK_HZ);
    RlsSetPoleAggressiveness(sp.aggressiveness);
    RunMove(&plant, p, countsPerRad, &seedGains, 12345, &positionTarget, 1);
    RunPause(&plant, p, countsPerRad, &seedGains, positionTarget, 500, 1);
    RunMove(&plant, p, countsPerRad, &seedGains, 67890, &positionTarget, 1);
    RunPause(&plant, p, countsPerRad, &seedGains, positionTarget, 500, 1);
    RunMove(&plant, p, countsPerRad, &seedGains, 24680, &positionTarget, 1);

    AutotuneResult result = RlsGetResult();
    if (result != AUTOTUNE_RESULT_OK) {
      if (perTrialCsv) {
        fprintf(perTrialCsv, "%d,%d,%d,,,%d,,,0,0\n", trial, velGainKpQ8, velGainKiQ8,
                before.maxErrCruise);
      }
      continue;
    }
    s.converged++;

    ControlGains tunedGains = seedGains;
    tunedGains.velGainKpQ8 = RlsGetKpResultQ8();
    tunedGains.velGainKiQ8 = RlsGetKiResultQ8();
    GainGlideState glide;
    GainGlideStart(&glide, ControlGetGains(), &tunedGains, sp.glideMs);
    RunGlide(&plant, p, countsPerRad, positionTarget, &glide);
    ControlReset();

    memset(&plant, 0, sizeof(plant));
    positionTarget = 0;
    MoveMetrics after = RunMove(&plant, p, countsPerRad, &tunedGains, 12345, &positionTarget, 0);
    int thisImproved = after.maxErrCruise < before.maxErrCruise;
    if (thisImproved) s.improved++;
    int thisBeatsKnownGood = after.maxErrCruise <= knownGoodCruise;
    if (thisBeatsKnownGood) s.beatsKnownGood++;
    if (perTrialCsv) {
      fprintf(perTrialCsv, "%d,%d,%d,%d,%d,%d,%d,%d,1,%d\n", trial, velGainKpQ8, velGainKiQ8,
              tunedGains.velGainKpQ8, tunedGains.velGainKiQ8, before.maxErrCruise, after.maxErrCruise,
              knownGoodCruise, thisBeatsKnownGood);
    }
  }
  return s;
}

typedef struct {
  int32_t foundKp, foundKi, beforeCruise, afterCruise;
  AutotuneResult result;
} FixedSeedResult;

/* Same before/learn/glide/after shape as one RunSweep() trial, but from a
 * SINGLE caller-chosen seed held fixed across aggressiveness levels --
 * answers a sharper question than RunSweep's randomized-seed counts:
 * starting from the exact known-good hand-tuned gains, does the candidate
 * RLS converges to at the chosen aggressiveness level actually track
 * BETTER than what the immediate neighboring levels would have produced
 * from that same starting point, not just "better on average across 100
 * random draws." */
static FixedSeedResult RunFromFixedSeed(const double *p, double countsPerRad, int32_t seedKp,
                                         int32_t seedKi, int32_t aggressiveness) {
  ControlGains seedGains = {
      .posGainQ8 = 16,
      .velGainKpQ8 = seedKp,
      .velGainKiQ8 = seedKi,
      .velocityClamp = 90,
      .torqueClampMin = -3276,
      .torqueClampMax = 3276,
  };
  AxisCommandState plant;
  int32_t positionTarget;

  ControlInit(&seedGains);
  memset(&plant, 0, sizeof(plant));
  positionTarget = 0;
  MoveMetrics before = RunMove(&plant, p, countsPerRad, &seedGains, 12345, &positionTarget, 0);

  ControlInit(&seedGains);
  memset(&plant, 0, sizeof(plant));
  positionTarget = 0;
  RlsInit();
  RlsStart(256, CONTROL_TICK_HZ);
  RlsSetPoleAggressiveness(aggressiveness);
  RunMove(&plant, p, countsPerRad, &seedGains, 12345, &positionTarget, 1);
  RunPause(&plant, p, countsPerRad, &seedGains, positionTarget, 500, 1);
  RunMove(&plant, p, countsPerRad, &seedGains, 67890, &positionTarget, 1);
  RunPause(&plant, p, countsPerRad, &seedGains, positionTarget, 500, 1);
  RunMove(&plant, p, countsPerRad, &seedGains, 24680, &positionTarget, 1);

  FixedSeedResult r = {0, 0, before.maxErrCruise, 0, RlsGetResult()};
  if (r.result != AUTOTUNE_RESULT_OK) return r;

  ControlGains tunedGains = seedGains;
  tunedGains.velGainKpQ8 = RlsGetKpResultQ8();
  tunedGains.velGainKiQ8 = RlsGetKiResultQ8();
  GainGlideState glide;
  GainGlideStart(&glide, ControlGetGains(), &tunedGains, 2000);
  RunGlide(&plant, p, countsPerRad, positionTarget, &glide);
  ControlReset();

  memset(&plant, 0, sizeof(plant));
  positionTarget = 0;
  MoveMetrics after = RunMove(&plant, p, countsPerRad, &tunedGains, 12345, &positionTarget, 0);
  r.foundKp = tunedGains.velGainKpQ8;
  r.foundKi = tunedGains.velGainKiQ8;
  r.afterCruise = after.maxErrCruise;
  return r;
}

int main(int argc, char **argv) {
  double p[8], countsPerRad;
  if (LoadModel(p, &countsPerRad) != 0) return 1;
  printf("loaded %s: countsPerRad=%.4f\n", MODEL_PATH, countsPerRad);

  /* DIAGNOSTIC: one representative run, printing RLS's own internal
   * theta_a/theta_b estimate (RLS.h's diagnostics-only getters) rather than
   * just this file's hand-derived expectation of the plant's torque->
   * velocity gain -- confirms whether the near-universal BAD_MEASUREMENT
   * above is RLS's divide-by-zero/sanity floor on theta_b specifically. */
  {
    ControlGains diagGains = {
        .posGainQ8 = 16, .velGainKpQ8 = 64000, .velGainKiQ8 = 4096,
        .velocityClamp = 90, .torqueClampMin = -3276, .torqueClampMax = 3276,
    };
    AxisCommandState plant;
    int32_t positionTarget = 0;
    ControlInit(&diagGains);
    memset(&plant, 0, sizeof(plant));
    RlsInit();
    RlsStart(256, CONTROL_TICK_HZ);
    RlsSetPoleAggressiveness(5); /* a^64 -- see RLS.h; this plant's own open-loop pole estimate
                                     sits close to 1, so the default a^2 target under-drives Kp/Ki
                                     (see this directory's README.md) */
    MoveMetrics mv = RunMove(&plant, p, countsPerRad, &diagGains, 12345, &positionTarget, 1);
    printf("DIAGNOSTIC: thetaA(Q8)=%d thetaB(Q8)=%d ticksObserved=%u pegTicks=%d maxErrCruise=%d result=%s\n",
           RlsGetThetaAQ8(), RlsGetThetaBQ8(), RlsGetTicksObserved(), mv.pegTicks, mv.maxErrCruise,
           ResultName(RlsGetResult()));
  }

  /* Fixed reference: the same PC-grid-search-selected gains
   * the independent bounded PC search exercised against both sim
   * and real hardware, at THIS harness's velocityClamp/torqueClamp. Each
   * trial's own random seed is mostly uninformative (a lot of random
   * (Pv,Ki) pairs already track tolerably by chance) -- this is the
   * baseline that actually matters: does RLS's converged candidate compete
   * with a known-good hand tune, not just beat an arbitrary random draw. */
  int32_t knownGoodCruise;
  {
    ControlGains knownGood = {
        .posGainQ8 = 16, .velGainKpQ8 = 64000, .velGainKiQ8 = 4096,
        .velocityClamp = 90, .torqueClampMin = -3276, .torqueClampMax = 3276,
    };
    AxisCommandState plant;
    int32_t positionTarget = 0;
    ControlInit(&knownGood);
    memset(&plant, 0, sizeof(plant));
    MoveMetrics mv = RunMove(&plant, p, countsPerRad, &knownGood, 12345, &positionTarget, 0);
    knownGoodCruise = mv.maxErrCruise;
    printf("known-good PC-search gains (Pv=64000 Iv=4096): maxErrCruise=%d\n", knownGoodCruise);
  }

  /* --sweep-csv <summary.csv> [detail.csv] [detailLevel]: writes the blog-post
   * data -- a summary row per sPoleAggressiveness level (0=a^2 .. 6=a^128) in
   * the first file, and the full 100-trial detail for ONE chosen level (5 =
   * a^64, this repo's shipped default, unless detailLevel says otherwise) in
   * the second -- see examples/rls_axis_command_v2/plot_sweep.py. */
  if (argc >= 3 && strcmp(argv[1], "--sweep-csv") == 0) {
    FILE *summaryCsv = fopen(argv[2], "w");
    FILE *detailCsv = (argc >= 4) ? fopen(argv[3], "w") : NULL;
    int32_t detailLevel = (argc >= 5) ? atoi(argv[4]) : 5;
    fprintf(summaryCsv, "aggressiveness,exponent,converged,improved,beats_known_good,trials\n");
    if (detailCsv) {
      fprintf(detailCsv,
              "trial,seed_kp_q8,seed_ki_q8,found_kp_q8,found_ki_q8,before_cruise,after_cruise,"
              "known_good_cruise,converged,beats_known_good\n");
    }
    for (int32_t level = 0; level <= 6; level++) { /* RLS_POLE_AGGRESSIVENESS_MAX; see RLS.h --
                                                         the blog writeup's exploratory data past
                                                         this point came from a temporarily-raised
                                                         clamp, not this shipped example */
      SweepParams sp = kDefaultSweepParams;
      sp.aggressiveness = level;
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp,
                                 level == detailLevel ? detailCsv : NULL);
      int exponent = 2;
      for (int32_t i = 0; i < level; i++) exponent *= 2; /* a^(2^(level+1)) */
      fprintf(summaryCsv, "%d,%d,%d,%d,%d,%d\n", level, exponent, s.converged, s.improved,
              s.beatsKnownGood, s.trials);
      printf("aggressiveness=%d (a^%d): %d/%d converged, %d/%d improved, %d/%d beat known-good\n", level,
             exponent, s.converged, s.trials, s.improved, s.converged, s.beatsKnownGood, s.converged);
    }
    fclose(summaryCsv);
    if (detailCsv) fclose(detailCsv);
    printf("wrote %s%s%s\n", argv[2], detailCsv ? " and " : "", argc >= 4 ? argv[3] : "");
    return 0;
  }

  /* --lambda-sweep-csv <out.csv>: same 100-random-seed sweep as --sweep-csv,
   * but holding pole-aggressiveness at its corrected default (5, a^64) and
   * varying the forgetting factor lambdaQ8 instead. RLS_DEFAULT_LAMBDA_Q8
   * (Protocol.c) ships at 256 (1.0, no forgetting) -- "the stable default"
   * per its own comment, never actually swept against the alternative of
   * forgetting older samples faster. */
  if (argc >= 3 && strcmp(argv[1], "--lambda-sweep-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "lambda_q8,lambda,converged,improved,beats_known_good,trials\n");
    const int32_t lambdas[] = {64, 96, 128, 160, 192, 224, 240, 256};
    for (size_t i = 0; i < sizeof(lambdas) / sizeof(lambdas[0]); i++) {
      SweepParams sp = kDefaultSweepParams;
      sp.lambdaQ8 = lambdas[i];
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
      fprintf(csv, "%d,%.4f,%d,%d,%d,%d\n", lambdas[i], lambdas[i] / 256.0, s.converged, s.improved,
              s.beatsKnownGood, s.trials);
      printf("lambda_q8=%d (%.4f): %d/%d converged, %d/%d improved, %d/%d beat known-good\n",
             lambdas[i], lambdas[i] / 256.0, s.converged, s.trials, s.improved, s.converged,
             s.beatsKnownGood, s.converged);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --lambda-multiseed-csv <out.csv>: the same lambda sweep repeated at 5
   * independent RNG seeds, to check whether --lambda-sweep-csv's single-seed
   * bump at lambda=0.875 is a real effect or sampling noise from reusing one
   * fixed set of 100 starting points. */
  if (argc >= 3 && strcmp(argv[1], "--lambda-multiseed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "rng_seed,lambda_q8,lambda,converged,improved,beats_known_good,trials\n");
    const int32_t lambdas[] = {64, 96, 128, 160, 192, 224, 240, 256};
    const uint32_t seeds[] = {0x9e3779b9u, 0x1u, 0xdeadbeefu, 0xc001d00du, 0x13572468u};
    for (size_t sidx = 0; sidx < sizeof(seeds) / sizeof(seeds[0]); sidx++) {
      for (size_t i = 0; i < sizeof(lambdas) / sizeof(lambdas[0]); i++) {
        SweepParams sp = kDefaultSweepParams;
        sp.lambdaQ8 = lambdas[i];
        sp.rngSeed = seeds[sidx];
        SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
        fprintf(csv, "0x%08x,%d,%.4f,%d,%d,%d,%d\n", seeds[sidx], lambdas[i], lambdas[i] / 256.0,
                s.converged, s.improved, s.beatsKnownGood, s.trials);
        printf("seed=0x%08x lambda_q8=%d: %d/%d converged, %d/%d beat known-good\n", seeds[sidx],
               lambdas[i], s.converged, s.trials, s.beatsKnownGood, s.converged);
      }
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --glide-sweep-csv <out.csv>: same shape, varying the gain-glide duration
   * instead. RLS_DEFAULT_GLIDE_MS (Protocol.c) ships at 2000ms -- "'resolves
   * to delightful' should be felt, not [measured]" per its own comment, a
   * stated-as-subjective default, not a swept one. */
  if (argc >= 3 && strcmp(argv[1], "--glide-sweep-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "glide_ms,converged,improved,beats_known_good,trials\n");
    const int32_t glides[] = {100, 250, 500, 1000, 1500, 2000, 3000, 5000};
    for (size_t i = 0; i < sizeof(glides) / sizeof(glides[0]); i++) {
      SweepParams sp = kDefaultSweepParams;
      sp.glideMs = glides[i];
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
      fprintf(csv, "%d,%d,%d,%d,%d\n", glides[i], s.converged, s.improved, s.beatsKnownGood, s.trials);
      printf("glide_ms=%d: %d/%d converged, %d/%d improved, %d/%d beat known-good\n",
             glides[i], s.converged, s.trials, s.improved, s.converged, s.beatsKnownGood, s.converged);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --glide-multiseed-csv <out.csv>: same idea as --lambda-multiseed-csv --
   * --glide-sweep-csv was already a flat null on one seed; confirms it stays
   * flat (not just "flat by coincidence of this seed") across independent
   * seeds too. */
  if (argc >= 3 && strcmp(argv[1], "--glide-multiseed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "rng_seed,glide_ms,converged,improved,beats_known_good,trials\n");
    const int32_t glides[] = {100, 250, 500, 1000, 1500, 2000, 3000, 5000};
    const uint32_t seeds[] = {0x9e3779b9u, 0x1u, 0xdeadbeefu, 0xc001d00du, 0x13572468u};
    for (size_t sidx = 0; sidx < sizeof(seeds) / sizeof(seeds[0]); sidx++) {
      for (size_t i = 0; i < sizeof(glides) / sizeof(glides[0]); i++) {
        SweepParams sp = kDefaultSweepParams;
        sp.glideMs = glides[i];
        sp.rngSeed = seeds[sidx];
        SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
        fprintf(csv, "0x%08x,%d,%d,%d,%d,%d\n", seeds[sidx], glides[i], s.converged, s.improved,
                s.beatsKnownGood, s.trials);
        printf("seed=0x%08x glide_ms=%d: %d/%d converged, %d/%d beat known-good\n", seeds[sidx],
               glides[i], s.converged, s.trials, s.beatsKnownGood, s.converged);
      }
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --bmax-multiseed-csv <out.csv>: same idea -- --bmax-sweep-csv was a flat
   * null across a 64x range on one seed; confirms across independent seeds. */
  if (argc >= 3 && strcmp(argv[1], "--bmax-multiseed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "rng_seed,b_max_at_ref_q8,converged,improved,beats_known_good,trials\n");
    const int32_t bmaxes[] = {8, 16, 32, 48, 64, 96, 128, 256, 512};
    const uint32_t seeds[] = {0x9e3779b9u, 0x1u, 0xdeadbeefu, 0xc001d00du, 0x13572468u};
    for (size_t sidx = 0; sidx < sizeof(seeds) / sizeof(seeds[0]); sidx++) {
      for (size_t i = 0; i < sizeof(bmaxes) / sizeof(bmaxes[0]); i++) {
        SweepParams sp = kDefaultSweepParams;
        sp.bMaxAtRefQ8 = bmaxes[i];
        sp.rngSeed = seeds[sidx];
        SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
        fprintf(csv, "0x%08x,%d,%d,%d,%d,%d\n", seeds[sidx], bmaxes[i], s.converged, s.improved,
                s.beatsKnownGood, s.trials);
        printf("seed=0x%08x b_max_at_ref_q8=%d: %d/%d converged, %d/%d beat known-good\n", seeds[sidx],
               bmaxes[i], s.converged, s.trials, s.beatsKnownGood, s.converged);
      }
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --deadzone-multiseed-csv <out.csv>: same idea -- --deadzone-sweep-csv
   * showed a real-looking downward trend past 1x on one seed; confirms it
   * across independent seeds (not just the shape, the fact that it holds). */
  if (argc >= 3 && strcmp(argv[1], "--deadzone-multiseed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "rng_seed,u_min,v_min,mult,converged,improved,beats_known_good,trials\n");
    const double mults[] = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
    const uint32_t seeds[] = {0x9e3779b9u, 0x1u, 0xdeadbeefu, 0xc001d00du, 0x13572468u};
    for (size_t sidx = 0; sidx < sizeof(seeds) / sizeof(seeds[0]); sidx++) {
      for (size_t i = 0; i < sizeof(mults) / sizeof(mults[0]); i++) {
        SweepParams sp = kDefaultSweepParams;
        sp.uMin = (int32_t)(50 * mults[i]);
        sp.vMin = (int32_t)(2 * mults[i]);
        if (sp.vMin < 1) sp.vMin = 1;
        sp.rngSeed = seeds[sidx];
        SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
        fprintf(csv, "0x%08x,%d,%d,%.4f,%d,%d,%d,%d\n", seeds[sidx], sp.uMin, sp.vMin, mults[i],
                s.converged, s.improved, s.beatsKnownGood, s.trials);
        printf("seed=0x%08x u_min=%d v_min=%d: %d/%d converged, %d/%d beat known-good\n", seeds[sidx],
               sp.uMin, sp.vMin, s.converged, s.trials, s.beatsKnownGood, s.converged);
      }
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --dither-multiseed-csv <out.csv>: same idea as --lambda-multiseed-csv --
   * --dither-sweep-csv's single-seed run showed a jump at 50% dither that
   * the lambda check taught us not to trust without independent seeds. */
  if (argc >= 3 && strcmp(argv[1], "--dither-multiseed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "rng_seed,dither_pct_q8,dither_pct,converged,improved,beats_known_good,trials\n");
    const int32_t dithers[] = {0, 1, 3, 7, 13, 26, 51, 102, 128};
    const uint32_t seeds[] = {0x9e3779b9u, 0x1u, 0xdeadbeefu, 0xc001d00du, 0x13572468u};
    for (size_t sidx = 0; sidx < sizeof(seeds) / sizeof(seeds[0]); sidx++) {
      for (size_t i = 0; i < sizeof(dithers) / sizeof(dithers[0]); i++) {
        SweepParams sp = kDefaultSweepParams;
        sp.ditherPctQ8 = dithers[i];
        sp.rngSeed = seeds[sidx];
        SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
        fprintf(csv, "0x%08x,%d,%.4f,%d,%d,%d,%d\n", seeds[sidx], dithers[i], dithers[i] / 256.0,
                s.converged, s.improved, s.beatsKnownGood, s.trials);
        printf("seed=0x%08x dither_pct_q8=%d: %d/%d converged, %d/%d beat known-good\n", seeds[sidx],
               dithers[i], s.converged, s.trials, s.beatsKnownGood, s.converged);
      }
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --bmax-sweep-csv <out.csv>: varies RLS_B_MAX_AT_REF_Q8, the b-estimate's
   * outer sanity ceiling -- explicitly commented "deliberately generous,
   * bench-TBD guess, not a measured value" in RLS.c, the most clearly
   * unvalidated default left after pole-aggressiveness/lambda/glide. Default
   * ships at 64; swept both tighter and looser. */
  if (argc >= 3 && strcmp(argv[1], "--bmax-sweep-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "b_max_at_ref_q8,converged,improved,beats_known_good,trials\n");
    const int32_t bmaxes[] = {8, 16, 32, 48, 64, 96, 128, 256, 512};
    for (size_t i = 0; i < sizeof(bmaxes) / sizeof(bmaxes[0]); i++) {
      SweepParams sp = kDefaultSweepParams;
      sp.bMaxAtRefQ8 = bmaxes[i];
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
      fprintf(csv, "%d,%d,%d,%d,%d\n", bmaxes[i], s.converged, s.improved, s.beatsKnownGood, s.trials);
      printf("b_max_at_ref_q8=%d: %d/%d converged, %d/%d beat known-good\n", bmaxes[i], s.converged,
             s.trials, s.beatsKnownGood, s.converged);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --deadzone-sweep-csv <out.csv>: scales RLS_U_MIN/RLS_V_MIN together (same
   * ratio, 50/2) by a multiplier -- RLS.c's own comment argues these are
   * correctly NOT tick-rate-scaled (a quantization-noise-floor role, same as
   * Autotune.c's AUTOTUNE_HYSTERESIS_COUNTS), a reasoned derivation rather
   * than an ad hoc guess, but never swept either. */
  if (argc >= 3 && strcmp(argv[1], "--deadzone-sweep-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "u_min,v_min,converged,improved,beats_known_good,trials\n");
    const double mults[] = {0.1, 0.25, 0.5, 1.0, 2.0, 4.0, 8.0, 16.0};
    for (size_t i = 0; i < sizeof(mults) / sizeof(mults[0]); i++) {
      SweepParams sp = kDefaultSweepParams;
      sp.uMin = (int32_t)(50 * mults[i]);
      sp.vMin = (int32_t)(2 * mults[i]);
      if (sp.vMin < 1) sp.vMin = 1;
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
      fprintf(csv, "%d,%d,%d,%d,%d,%d\n", sp.uMin, sp.vMin, s.converged, s.improved, s.beatsKnownGood,
              s.trials);
      printf("u_min=%d v_min=%d: %d/%d converged, %d/%d beat known-good\n", sp.uMin, sp.vMin,
             s.converged, s.trials, s.beatsKnownGood, s.converged);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --dither-sweep-csv <out.csv>: varies RLS_DITHER_PCT_Q8. RLS.c's own
   * comment already claims "as little as 1% already fully corrects the
   * closed-loop identification bias... flat from there through 50%" --
   * checked here with the same formal before/after sweep as everything
   * else, not just the ad hoc check that comment describes. */
  if (argc >= 3 && strcmp(argv[1], "--dither-sweep-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "dither_pct_q8,dither_pct,converged,improved,beats_known_good,trials\n");
    const int32_t dithers[] = {0, 1, 3, 7, 13, 26, 51, 102, 128};
    for (size_t i = 0; i < sizeof(dithers) / sizeof(dithers[0]); i++) {
      SweepParams sp = kDefaultSweepParams;
      sp.ditherPctQ8 = dithers[i];
      SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, sp, NULL);
      fprintf(csv, "%d,%.4f,%d,%d,%d,%d\n", dithers[i], dithers[i] / 256.0, s.converged, s.improved,
              s.beatsKnownGood, s.trials);
      printf("dither_pct_q8=%d (%.1f%%): %d/%d converged, %d/%d beat known-good\n", dithers[i],
             100.0 * dithers[i] / 256.0, s.converged, s.trials, s.beatsKnownGood, s.converged);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --fixed-seed-csv <out.csv>: RunFromFixedSeed across every aggressiveness
   * level, held to the SAME known-good hand-tuned starting gains each time --
   * the blog writeup's "same starting point, does a^64 actually track better
   * than its neighbors" figure, sharper than --sweep-csv's randomized-seed
   * counts. */
  if (argc >= 3 && strcmp(argv[1], "--fixed-seed-csv") == 0) {
    FILE *csv = fopen(argv[2], "w");
    fprintf(csv, "aggressiveness,exponent,converged,before_cruise,after_cruise,found_kp_q8,found_ki_q8\n");
    for (int32_t level = 0; level <= 6; level++) {
      FixedSeedResult r = RunFromFixedSeed(p, countsPerRad, 64000, 4096, level);
      int exponent = 2;
      for (int32_t i = 0; i < level; i++) exponent *= 2;
      fprintf(csv, "%d,%d,%d,%d,%d,%d,%d\n", level, exponent, r.result == AUTOTUNE_RESULT_OK,
              r.beforeCruise, r.afterCruise, r.foundKp, r.foundKi);
      printf("aggressiveness=%d (a^%d): converged=%d before=%d after=%d foundKp=%d foundKi=%d\n",
             level, exponent, r.result == AUTOTUNE_RESULT_OK, r.beforeCruise, r.afterCruise,
             r.foundKp, r.foundKi);
    }
    fclose(csv);
    printf("wrote %s\n", argv[2]);
    return 0;
  }

  /* --fixed-gains <velKpQ8> <velKiQ8> [seed] [velocityClamp] [torqueClampMax] [posGainQ8]:
   * run this model's own RunMove (no RLS, feedRls=0) with an explicit
   * velocity-loop gain pair, for direct comparison against a real board's
   * own measured tracking error running the SAME applied gains over the
   * SAME envelope -- see the hardware writeup. Prints
   * maxErrAccel/maxErrCruise/pegTicks, nothing else. Trailing args default
   * to this file's original fixed envelope (90/3276/16) when omitted. */
  if (argc >= 4 && strcmp(argv[1], "--fixed-gains") == 0) {
    int32_t velKpQ8 = atoi(argv[2]);
    int32_t velKiQ8 = atoi(argv[3]);
    uint32_t seed = (argc >= 5) ? (uint32_t)strtoul(argv[4], NULL, 10) : 12345U;
    int32_t velocityClamp = (argc >= 6) ? atoi(argv[5]) : 90;
    int32_t torqueClampMax = (argc >= 7) ? atoi(argv[6]) : 3276;
    int32_t posGainQ8 = (argc >= 8) ? atoi(argv[7]) : 16;
    ControlGains gains = {
        .posGainQ8 = posGainQ8, .velGainKpQ8 = velKpQ8, .velGainKiQ8 = velKiQ8,
        .velocityClamp = velocityClamp, .torqueClampMin = -torqueClampMax, .torqueClampMax = torqueClampMax,
    };
    AxisCommandState plant;
    int32_t positionTarget = 0;
    ControlInit(&gains);
    memset(&plant, 0, sizeof(plant));
    MoveMetrics mv = RunMove(&plant, p, countsPerRad, &gains, seed, &positionTarget, 0);
    printf("fixed-gains velKp=%d(%.3f) velKi=%d(%.3f) seed=%u clamp=%d torqueMax=%d posGain=%d: "
           "maxErrAccel=%d maxErrCruise=%d pegTicks=%d\n",
           velKpQ8, velKpQ8 / 256.0, velKiQ8, velKiQ8 / 256.0, seed, velocityClamp, torqueClampMax, posGainQ8,
           mv.maxErrAccel, mv.maxErrCruise, mv.pegTicks);
    return 0;
  }

  /* --readme-demo <out.json> <velKpQ8> <velKiQ8> <velocityClamp> <torqueClampMax> <posGainQ8>
   *   [stepAmplitude] [stepTicks] [rampPeak] [rampAccelCountsPerTickPerSec]:
   * Mirrors tools/capture_moves.py's two canonical moves -- a sharp,
   * unramped position step (same idea as `test square position`: feed the
   * full step error straight into the cascade) and a triangular velocity
   * ramp (linear ramp up to rampPeak, hold, ramp back down) -- against
   * THIS model instead of real hardware, dumping one JSON frame per tick
   * in the exact schema tools/monitor_frame.FrameSync produces, so
   * tools/plot_telemetry.py renders this sim run with the SAME code (and
   * so the same visual language) as the real analog-servo-f103 bench
   * capture it's being compared against. */
  if (argc >= 7 && (strcmp(argv[1], "--readme-demo") == 0 ||
                    strcmp(argv[1], "--readme-demo-no-reference") == 0 ||
                    strcmp(argv[1], "--readme-demo-legacy") == 0)) {
    const char *outPath = argv[2];
    int32_t velKpQ8 = atoi(argv[3]);
    int32_t velKiQ8 = atoi(argv[4]);
    int32_t velocityClamp = atoi(argv[5]);
    int32_t torqueClampMax = atoi(argv[6]);
    int32_t posGainQ8 = (argc >= 8) ? atoi(argv[7]) : 16;
    int32_t stepAmplitude = (argc >= 9) ? atoi(argv[8]) : 1024;
    int32_t stepTicks = (argc >= 10) ? atoi(argv[9]) : 400;
    int32_t rampPeak = (argc >= 11) ? atoi(argv[10]) : 28;
    /* Same units/semantics as real firmware's trajectory.accel_limit
     * (encoder_count/tick/second) -- NOT counts/tick^2 -- so a given value
     * here produces the identical ramp duration as the same value passed
     * to capture_moves.py's --velocity-accel. */
    int32_t rampAccelCountsPerTickPerSec = (argc >= 12) ? atoi(argv[11]) : 250;

    ControlGains gains = {
        .posGainQ8 = posGainQ8, .velGainKpQ8 = velKpQ8, .velGainKiQ8 = velKiQ8,
        .velocityClamp = velocityClamp, .torqueClampMin = -torqueClampMax, .torqueClampMax = torqueClampMax,
    };
    AxisCommandState plant;
    memset(&plant, 0, sizeof(plant));
    ControlInit(&gains);

    FILE *out = fopen(outPath, "w");
    fprintf(out, "[\n");
    int legacy = strcmp(argv[1], "--readme-demo-legacy") == 0;
    AxisDemo demo = {.first=1, .encoderFeedback=!legacy,
                     .velocityReference=strcmp(argv[1], "--readme-demo")==0};
    int32_t positionTarget = 0;

    /* Move 1: sharp position step, no ramp -- positionTarget jumps once.
     * A short flat pre-roll at target=0 first, so the plot shows an actual
     * step edge (both torque and position starting from a settled zero
     * baseline) instead of frame 0 already mid-transient. Mirrors real
     * hardware's `test square position <amp> <half> <half>` exactly: that
     * signal only runs for ONE half-period (SIGNAL_SQUARE alternates
     * +amplitude/-amplitude every half-period, but duration==half-period
     * means it stops after the first +amplitude half and is never active
     * long enough to reach the -amplitude half) -- once it stops, control
     * falls through to the normal cascade, which still targets the
     * pre-test (~0) position.command and pulls the axis back down. So
     * this is a step UP held for stepTicks, then a return to 0, not a
     * held stepTicks*2 plateau. */
    positionTarget = 0;
    for (int i = 0; i < 20; i++)
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,0,"preroll");
    positionTarget = stepAmplitude;
    for (int i = 0; i < stepTicks; i++)
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,0,"step");
    positionTarget = 0;
    for (int i = 0; i < stepTicks; i++)
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,0,"step-return");

    /* 0.3s settle gap, matching capture_moves.py's own pacing between moves. */
    for (int i = 0; i < 300; i++)
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,0,"gap");

    /* Move 2: triangular velocity ramp -- linear ramp up to rampPeak, ramp
     * back down, same shape (and same accel units) as Ramp.c's own profile
     * driving velocity.command on real hardware. cmdVelD accumulates as a
     * double so a non-round accel doesn't get truncated away by integer
     * division the way rampAccelPerTick's old counts/tick^2 framing did. */
    double accelPerTick = (double)rampAccelCountsPerTickPerSec / CONTROL_TICK_HZ;
    double cmdVelD = 0.0;
    int32_t cmdVel = 0;
    int rampTicks = (accelPerTick > 0) ? (int)(rampPeak / accelPerTick) : 0;
    for (int i = 0; i < rampTicks; i++) {
      cmdVelD += accelPerTick;
      cmdVel = (int32_t)lround(cmdVelD);
      positionTarget += cmdVel;
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,cmdVel,"ramp-up");
    }
    for (int i = 0; i < rampTicks; i++) {
      cmdVelD -= accelPerTick;
      cmdVel = (int32_t)lround(cmdVelD);
      positionTarget += cmdVel;
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,cmdVel,"ramp-down");
    }
    for (int i = 0; i < 400; i++) {
      positionTarget += cmdVel;
      DemoTick(out,&demo,&plant,p,countsPerRad,positionTarget,cmdVel,"ramp-hold");
    }
    fprintf(out, "\n]\n");
    fclose(out);
    printf("wrote %s (velKp=%.3f velKi=%.3f clamp=%d torqueMax=%d posGain=%.4f step=%d ramp_peak=%d)\n",
           outPath, velKpQ8 / 256.0, velKiQ8 / 256.0, velocityClamp, torqueClampMax, posGainQ8 / 256.0,
           stepAmplitude, rampPeak);
    return 0;
  }

  SweepSummary s = RunSweep(p, countsPerRad, knownGoodCruise, kDefaultSweepParams, NULL);
  printf("\n%d/%d starting points converged (AUTOTUNE_RESULT_OK); %d/%d of those improved cruise "
         "tracking vs. their own seed; %d/%d matched or beat the known-good PC-search gains "
         "(maxErrCruise=%d)\n",
         s.converged, s.trials, s.improved, s.converged, s.beatsKnownGood, s.converged, knownGoodCruise);
  printf("DIAGNOSTIC -- sim-only, velocityClamp/torqueClamp extrapolate beyond recorded hardware "
         "speeds; no claim here about hardware agreement, see docs/characterization_fitter.md\n");
  return 0;
}
