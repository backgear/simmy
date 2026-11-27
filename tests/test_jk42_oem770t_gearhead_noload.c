/* Pin the canonical JK42 preset and its stepping contract. These checks
 * prevent model regressions; they do not independently validate the fit. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "simmy.h"
#include "simmy_json.h"

#define PRESET "models/jk42_oem770t_gearhead_noload.json"

typedef struct {
  SimmyChainParams chain;
  SimmyAnalogChainParams drive;
  SimmyChainState state;
  SimmyAnalogChainState driveState;
  int steps;
} Plant;

static void Load(Plant *p) {
  char err[256];
  assert(SimmyChainParamsFromJsonFile(PRESET, &p->chain, err, sizeof err) == 0);
  assert(SimmyAnalogChainParamsFromJsonFile(PRESET, &p->drive, err, sizeof err) == 0);
  FILE *f = fopen(PRESET, "rb");
  assert(f);
  char text[16384];
  size_t n = fread(text, 1, sizeof text - 1, f);
  assert(!ferror(f) && feof(f));
  fclose(f);
  text[n] = 0;
  SimmyJsonValue *root = SimmyJsonParse(text, err, sizeof err);
  assert(root);
  p->steps = (int)SimmyJsonNumber(SimmyJsonObjectGet(root, "analogStepsPerTick"), 0);
  SimmyJsonFree(root);
  assert(p->steps == 10);
  SimmyChainInit(&p->state, &p->chain);
  SimmyAnalogChainInit(&p->driveState, 1);
}

static void Tick(Plant *p, double torque) {
  for (int i = 0; i < p->steps; i++) {
    double shaft = SimmyAnalogChainStep(&p->driveState, &p->drive, torque);
    SimmyChainStepDt(&p->state, &p->chain, shaft, 1.0 / p->steps);
  }
  for (int i = 0; i < p->chain.bodyCount; i++) {
    assert(isfinite(p->state.bodies[i].position));
    assert(isfinite(p->state.bodies[i].velocity));
  }
}

static void test_parameters(void) {
  Plant p;
  Load(&p);
  assert(p.chain.bodyCount == 3 && p.chain.feedbackBody == 0 && p.chain.subSteps == 4);
  const double inertia[] = {2437.7814793835264, 203.30285554898816, 150.01880641672838};
  const double stiction[] = {1151.1524778717223, 538.6149406041748, 397.4482812327498};
  for (int i = 0; i < 3; i++) {
    assert(fabs(p.chain.bodies[i].inertia - inertia[i]) < 1e-9);
    assert(fabs(p.chain.bodies[i].stictionLimit - stiction[i]) < 1e-9);
    assert(p.chain.bodies[i].stictionLimit > p.chain.bodies[i].coulombLevel);
  }
  assert(fabs(p.chain.bodies[0].ripplePhase - 0.6961829098009289) < 1e-12);
  assert(fabs(p.chain.couplings[0].springK - 27204.5540239571) < 1e-8);
  assert(fabs(p.chain.couplings[1].springK - 1667.274376965682) < 1e-8);
  assert(fabs(p.chain.couplings[0].backlashGap - 17.022855880354513) < 1e-10);
  assert(fabs(p.chain.couplings[1].backlashGap - 17.169837688537093) < 1e-10);
  assert(p.drive.stageCount == 2);
  assert(p.drive.stages[0].type == SIMMY_STAGE_DELAY);
  assert(fabs(p.drive.stages[0].param0 - 2.0963075205582764) < 1e-12);
  assert(p.drive.stages[1].type == SIMMY_STAGE_LAG);
  assert(fabs(p.drive.stages[1].param0 - 0.7087532879460348) < 1e-12);
}

static void test_rest_and_bidirectional_pulses(void) {
  for (int direction = -1; direction <= 1; direction += 2) {
    Plant p;
    Load(&p);
    for (int i = 0; i < 1000; i++) Tick(&p, 0);
    for (int i = 0; i < 3; i++) assert(p.state.bodies[i].position == 0);
    for (int i = 0; i < 60; i++) Tick(&p, direction * 3000);
    assert(direction * p.state.bodies[0].position > 100);
    assert(direction * p.state.bodies[2].position > 50);
    for (int i = 0; i < 1000; i++) Tick(&p, 0);
    for (int i = 0; i < 3; i++) assert(fabs(p.state.bodies[i].velocity) < 0.01);
  }
}

static void test_bounded_pd_move(void) {
  Plant p;
  Load(&p);
  int32_t previous = 0;
  for (int i = 0; i < 2000; i++) {
    int32_t position = SimmyChainFeedbackPositionCounts(&p.state, &p.chain);
    int32_t velocity = position - previous;
    previous = position;
    double torque = 100.0 * (1000 - position) - 1500.0 * velocity;
    if (torque > 30000) torque = 30000;
    if (torque < -30000) torque = -30000;
    Tick(&p, torque);
    assert(fabs(p.state.bodies[0].position) < 2000);
  }
  assert(abs(SimmyChainFeedbackPositionCounts(&p.state, &p.chain) - 1000) <= 25);
}

int main(void) {
  test_parameters();
  test_rest_and_bidirectional_pulses();
  test_bounded_pd_move();
  puts("All jk42_oem770t_gearhead_noload preset tests passed.");
  return 0;
}
