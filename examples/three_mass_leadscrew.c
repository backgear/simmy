/*
 * A JSON-driven THREE-body chain (examples/configs/three_mass_leadscrew.json):
 * motor -> belt -> lead screw -> nut (with backlash) -> table. Same
 * SimmyControllerFn demo PD controller as basic_two_mass.c (gains
 * re-picked for this config -- a gain set is only ever valid for the
 * plant it was matched to, see the README).
 *
 * Also demonstrates reading a body OTHER than the feedback one purely
 * for measurement: prints the table's (body 2) position alongside the
 * motor's, so you can see how well motor-side-only control tracks at
 * the load end of the chain -- exactly the comparison this is for (see
 * simmy.h's header comment on feedback-vs-measurement).
 *
 * Run from the repo root: ./build/three_mass_leadscrew
 */
#include <stdio.h>
#include <stdlib.h>

#include "simmy.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

static int32_t DemoPdController(void *userData, int32_t targetPosition, int32_t actualPosition,
                                 int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  (void)dtCycles;
  int64_t posError = (int64_t)targetPosition - actualPosition;
  int64_t command = posError - 6 * (int64_t)actualVelocity;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  return (int32_t)command;
}

int main(void) {
  const char *configPath = "examples/configs/three_mass_leadscrew.json";
  SimmyChainParams params;
  SimmyAnalogChainParams analogParams;
  char err[256];

  if (SimmyChainParamsFromJsonFile(configPath, &params, err, sizeof(err)) != 0) {
    fprintf(stderr, "failed to load %s: %s\n", configPath, err);
    return 1;
  }
  if (SimmyAnalogChainParamsFromJsonFile(configPath, &analogParams, err, sizeof(err)) != 0) {
    fprintf(stderr, "failed to load analog chain from %s: %s\n", configPath, err);
    return 1;
  }

  SimmyChainState state;
  SimmyChainInit(&state, &params);
  SimmyAnalogChainState analogState;
  SimmyAnalogChainInit(&analogState, 1);

  SimmyControllerFn controller = DemoPdController;
  int32_t target = 5000;
  int ticks = 4000;
  int tableBody = 2;

  printf("tick,target,motorPosition,tableBodyPosition,velocity,torque\n");
  for (int i = 0; i < ticks; i++) {
    int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    int32_t torque = controller(NULL, target, position, velocity, 32000);
    double force = SimmyAnalogChainStep(&analogState, &analogParams, (double)torque);

    int32_t tablePosition = SimmyChainBodyPositionCounts(&state, tableBody);   /* measurement only */
    printf("%d,%d,%d,%d,%d,%d\n", i, target, position, tablePosition, velocity, torque);

    SimmyChainStep(&state, &params, force);
  }

  int32_t motorError = target - SimmyChainFeedbackPositionCounts(&state, &params);
  int32_t tableError = target - SimmyChainBodyPositionCounts(&state, tableBody);
  fprintf(stderr, "final following error: motor %d counts, table %d counts (the gap between them "
                  "is exactly what motor-side-only feedback can't see)\n",
          Abs32(motorError), Abs32(tableError));
  return 0;
}
