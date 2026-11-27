/*
 * models/sm232ae_tq10_noload.json: the preset loads, its parameters are the
 * documented fit, and its open-loop and closed-loop responses look like
 * the bench capture it was fitted to (docs/sm232ae_tq10_noload.md). These
 * pin the preset; they are not evidence that the fit is right.
 * Run from the repo root: make test
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "simmy.h"
#include "simmy_json.h"

#define PRESET "models/sm232ae_tq10_noload.json"

typedef struct {
  SimmyChainParams chain;
  SimmyAnalogChainParams drive;
  SimmyChainState state;
  SimmyAnalogChainState driveState;
  int stepsPerTick;
} Plant;

static void LoadPlant(Plant *p, uint32_t seed) {
  char err[256] = {0};
  assert(SimmyChainParamsFromJsonFile(PRESET, &p->chain, err, sizeof(err)) == 0);
  assert(SimmyAnalogChainParamsFromJsonFile(PRESET, &p->drive, err, sizeof(err)) == 0);
  SimmyChainInit(&p->state, &p->chain);
  SimmyAnalogChainInit(&p->driveState, seed);
  p->stepsPerTick = 10;
}

/* One 1 ms tick: the preset's analog chain runs stepsPerTick times per tick. */
static void Tick(Plant *p, double torque) {
  for (int i = 0; i < p->stepsPerTick; i++) {
    double shaft = SimmyAnalogChainStep(&p->driveState, &p->drive, torque);
    SimmyChainStepDt(&p->state, &p->chain, shaft, 1.0 / p->stepsPerTick);
  }
}

static void test_preset_loads_with_documented_parameters(void) {
  Plant p;
  LoadPlant(&p, 1);
  const SimmyBodyParams *b = &p.chain.bodies[0];
  assert(p.chain.bodyCount == 1 && p.chain.feedbackBody == 0);
  assert(fabs(b->inertia - 4439.5) < 0.1);
  assert(fabs(b->friction - 4.634) < 0.001);
  assert(fabs(b->coulombLevel - 121.46) < 0.01);
  assert(fabs(b->coulombPerForce - 0.05402) < 1e-5);
  assert(fabs(b->stictionLimit - 1134.84) < 0.01);
  assert(b->gravityBias == 0 && b->rippleAmplitude == 0);

  assert(p.drive.stageCount == 4);
  assert(p.drive.stages[0].type == SIMMY_STAGE_DELAY);
  assert(fabs(p.drive.stages[0].param0 - 0.6201) < 1e-4);     /* 0.062 tick at 10 steps/tick */
  assert(p.drive.stages[1].type == SIMMY_STAGE_SLEW);
  assert(fabs(p.drive.stages[1].param0 - 7138.74) < 0.01);    /* 71387 torque/tick */
  assert(p.drive.stages[2].type == SIMMY_STAGE_LAG);
  assert(fabs(p.drive.stages[2].param0 - (1.0 - exp(-1.0 / 0.764874437124817))) < 1e-9);
  assert(p.drive.stages[3].type == SIMMY_STAGE_NOISE);
  assert(fabs(p.drive.stages[3].param0 - 2253.41) < 0.01);

  /* Metadata the C loaders don't read. */
  FILE *f = fopen(PRESET, "rb");
  assert(f);
  static char text[16384];
  size_t n = fread(text, 1, sizeof(text) - 1, f);
  fclose(f);
  text[n] = '\0';
  char err[256];
  SimmyJsonValue *root = SimmyJsonParse(text, err, sizeof(err));
  assert(root);
  assert(SimmyJsonNumber(SimmyJsonObjectGet(root, "analogStepsPerTick"), 0) == 10.0);
  assert(strcmp(SimmyJsonString(SimmyJsonObjectGet(root, "name"), ""), "sm232ae_tq10_noload") == 0);
  assert(strcmp(SimmyJsonString(SimmyJsonObjectGet(root, "status"), ""),
                "fitted to one bench capture, not independently validated") == 0);
  SimmyJsonFree(root);
}

/* Full-scale command from rest, then reversed: accelerates at about the
 * bench's 6.4-6.6 counts/tick^2 and brakes faster, as the bench did
 * (7.3-7.8). Noise stage dropped so the numbers are exact. */
static void test_preset_open_loop_accel_and_braking(void) {
  Plant p;
  LoadPlant(&p, 1);
  p.drive.stageCount = 3;
  for (int i = 0; i < 3; i++) Tick(&p, 30000);
  double v0 = p.state.bodies[0].velocity;
  for (int i = 0; i < 8; i++) Tick(&p, 30000);
  double accel = (p.state.bodies[0].velocity - v0) / 8;
  assert(isfinite(accel) && accel > 6.0 && accel < 6.8);

  for (int i = 0; i < 3; i++) Tick(&p, -30000);
  v0 = p.state.bodies[0].velocity;
  for (int i = 0; i < 5; i++) Tick(&p, -30000);
  double brake = (v0 - p.state.bodies[0].velocity) / 5;
  assert(brake > 6.9 && brake < 7.7);
  assert(brake > accel + 0.5);
}

/* Without noise, a command below the stiction limit never moves the shaft. */
static void test_preset_stiction_holds_small_command(void) {
  Plant p;
  LoadPlant(&p, 1);
  p.drive.stageCount = 3;
  for (int i = 0; i < 1000; i++) Tick(&p, 1000);
  assert(p.state.bodies[0].position == 0.0 && p.state.bodies[0].velocity == 0.0);
}

/* Closed loop through the full preset (noise on), integer encoder
 * feedback: a plain PD move of 1000 counts stays finite and ends near
 * the target. */
static void test_preset_closed_loop_pd_move_is_sane(void) {
  Plant p;
  LoadPlant(&p, 7);
  int32_t previous = 0;
  for (int i = 0; i < 1000; i++) {
    int32_t position = SimmyChainFeedbackPositionCounts(&p.state, &p.chain);
    int32_t velocity = position - previous;
    previous = position;
    double torque = 100.0 * (1000 - position) - 1500.0 * velocity;
    if (torque > 30000) torque = 30000;
    if (torque < -30000) torque = -30000;
    Tick(&p, torque);
    assert(isfinite(p.state.bodies[0].position) && isfinite(p.state.bodies[0].velocity));
    assert(fabs(p.state.bodies[0].position) < 2000);
  }
  int32_t final = SimmyChainFeedbackPositionCounts(&p.state, &p.chain);
  assert(abs(final - 1000) <= 20);
}

int main(void) {
  test_preset_loads_with_documented_parameters();
  test_preset_open_loop_accel_and_braking();
  test_preset_stiction_holds_small_command();
  test_preset_closed_loop_pd_move_is_sane();
  printf("All sm232ae_tq10_noload preset tests passed.\n");
  return 0;
}
