/*
 * A JSON-driven two-body chain (examples/configs/two_mass.json), driven
 * by a small demo PD controller -- an ordinary SimmyControllerFn
 * implementation, same calling convention a real controller linked in
 * from elsewhere would use (see examples/analog_servo_integration/).
 *
 * Prints one CSV-ish line per tick to stdout: tick,target,position,
 * velocity,torque. Run from the repo root so the relative config path
 * resolves: ./build/basic_two_mass
 */
#include <stdio.h>
#include <stdlib.h>

#include "simmy.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* Needs the velocity (damping) term -- a frictionless-ish mass under
 * pure position-proportional control is only marginally stable and
 * rings instead of settling; ordinary control theory, not a simmy
 * quirk. Gains hand-picked for THIS config via a step-response check
 * (see the repo's development notes) -- not tuned for any other config. */
static int32_t DemoPdController(void *userData, int32_t targetPosition, int32_t actualPosition,
                                 int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  (void)dtCycles;
  int64_t posError = (int64_t)targetPosition - actualPosition;
  int64_t command = posError - 10 * (int64_t)actualVelocity;
  if (command > 1000) command = 1000;
  if (command < -1000) command = -1000;
  return (int32_t)command;
}

int main(void) {
  const char *configPath = "examples/configs/two_mass.json";
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

  printf("tick,target,position,velocity,torque\n");
  for (int i = 0; i < ticks; i++) {
    int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    int32_t torque = controller(NULL, target, position, velocity, 32000);
    double force = SimmyAnalogChainStep(&analogState, &analogParams, (double)torque);

    printf("%d,%d,%d,%d,%d\n", i, target, position, velocity, torque);

    SimmyChainStep(&state, &params, force);
  }

  int32_t finalError = target - SimmyChainFeedbackPositionCounts(&state, &params);
  fprintf(stderr, "final following error: %d counts (abs %d)\n", finalError, Abs32(finalError));
  return 0;
}
