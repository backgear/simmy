/*
 * A REVERSING move through the three-body lead-screw chain
 * (examples/configs/three_mass_leadscrew.json) -- the whole point of a
 * bidirectional move: the nut coupling's backlash gap only shows up on
 * a direction change (see couplings[1].backlashGap in that config). A
 * one-way move never crosses the dead zone at all.
 *
 * Uses its own config (examples/configs/bidirectional_move.json) --
 * same chain as three_mass_leadscrew.json, but with a bigger nut
 * backlash gap specifically so the dead-zone crossing lasts long enough
 * to be obvious here (the default gap only shows 1-2 ticks of it).
 *
 * Drives a SimmyPulseTrainState at +rate for half the run, then -rate
 * for the other half (a signed rate -- see SimmyPulseTrainTickDelta),
 * same SimmyControllerFn demo PD controller as the other examples.
 *
 * Prints the nut's relative displacement each tick alongside the usual
 * columns, computed externally from the SAME public fields the library
 * itself uses (couplings[1].backlashGap, bodies[1]/[2] position) --
 * no special introspection API needed, just reading public state.
 *
 * Run from the repo root: ./build/bidirectional_move
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "simmy.h"

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
  const char *configPath = "examples/configs/bidirectional_move.json";
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
  SimmyPulseTrainState pulseState;
  SimmyPulseTrainInit(&pulseState, 12345);

  SimmyControllerFn controller = DemoPdController;
  double rateEdgesPerSec = 2500.0;
  int ticksPerLeg = 2000;
  int ticks = ticksPerLeg * 2;
  int32_t positionTarget = 0;
  double nutBacklashGap = params.couplings[1].backlashGap;

  printf("tick,direction,target,motorPosition,tableBodyPosition,nutRelativeDisplacement,inNutDeadZone\n");
  for (int i = 0; i < ticks; i++) {
    double rate = (i < ticksPerLeg) ? rateEdgesPerSec : -rateEdgesPerSec;
    int32_t delta = SimmyPulseTrainTickDelta(&pulseState, rate, 1000.0, 0.0);
    positionTarget += delta;

    int32_t motorPosition = SimmyChainFeedbackPositionCounts(&state, &params);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    int32_t torque = controller(NULL, positionTarget, motorPosition, velocity, 32000);
    double force = SimmyAnalogChainStep(&analogState, &analogParams, (double)torque);

    int32_t tableBodyPosition = SimmyChainBodyPositionCounts(&state, 2);
    double nutRelativeDisplacement = state.bodies[1].position - state.bodies[2].position;
    int inDeadZone = fabs(nutRelativeDisplacement) < nutBacklashGap;

    printf("%d,%s,%d,%d,%d,%.3f,%d\n", i, i < ticksPerLeg ? "fwd" : "rev", positionTarget,
           motorPosition, tableBodyPosition, nutRelativeDisplacement, inDeadZone);

    SimmyChainStep(&state, &params, force);
  }

  fprintf(stderr, "bidirectional move complete -- check the inNutDeadZone column around "
                  "tick %d (the reversal point) for the backlash dead-zone crossing\n",
          ticksPerLeg);
  return 0;
}
