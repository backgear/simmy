/*
 * Links the REAL firmware/src/Control.c from the sibling analog-servo
 * repo (read-only, by relative path -- see ../../../analog-servo) and
 * drives it through a simmy chain, via the SAME SimmyControllerFn
 * convention the plain examples' demo controller uses -- wrapping
 * ControlTick() is a direct passthrough, not a special adapter.
 *
 * Replays the exact scenario analog-servo's own
 * test_prototype_nema23_leadscrew_preset_jittered_is_stable already
 * calibrated (firmware/test/test_stepdir_control/test_stepdir_control.c):
 * a ramped, jittered pulse train through the two-body config that
 * mirrors analog-servo's kNema23Leadscrew* preset (examples/configs/
 * two_mass.json uses the identical numbers). The resulting maxErrAccel/
 * maxErrCruise/pegTicks should land comfortably inside that test's own
 * bounds (<=450 / <=300 / <=20) -- the concrete tie-back proving this
 * is genuine control-loop linkage, not just a demo stand-in.
 *
 * Only builds/runs via `make integration` (requires the ../analog-servo
 * sibling repo; see the Makefile). Run from the simmy repo root.
 */
#include <stdio.h>

#include "Control.h"
#include "simmy.h"

static int32_t Abs32(int32_t v) { return v < 0 ? -v : v; }

/* The entire adapter: wrapping a real production control cascade in
 * simmy's controller convention is this short. */
static int32_t ControlAdapter(void *userData, int32_t targetPosition, int32_t actualPosition,
                               int32_t actualVelocity, uint32_t dtCycles) {
  (void)userData;
  return ControlTick(targetPosition, actualPosition, actualVelocity, dtCycles);
}

int main(void) {
  ControlGains gains = {
    .posGainQ8 = 32, .velGainKpQ8 = 1024, .velGainKiQ8 = 1,
    .velocityClamp = 200000, .torqueClampMin = -1000, .torqueClampMax = 1000,
  };
  ControlInit(&gains);

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
  SimmyPulseTrainState pulseState;
  SimmyPulseTrainInit(&pulseState, 12345);

  SimmyControllerFn controller = ControlAdapter;   /* the real firmware cascade, linked in */

  double targetRate = 2500.0, rampSeconds = 1.0, jitterFraction = 0.3;
  int accelTicks = 1000, cruiseTicks = 1000, ticks = accelTicks + cruiseTicks;
  double accelRatePerSec2 = targetRate / rampSeconds;
  int32_t positionTarget = 0;
  int32_t maxErrAccel = 0, maxErrCruise = 0, pegTicks = 0;

  for (int i = 0; i < ticks; i++) {
    double rate = (i < accelTicks) ? (accelRatePerSec2 * i / 1000.0) : targetRate;
    int32_t delta = SimmyPulseTrainTickDelta(&pulseState, rate, 1000.0, jitterFraction);
    positionTarget += delta;

    int32_t position = SimmyChainFeedbackPositionCounts(&state, &params);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(&state, &params);
    int32_t torque = controller(NULL, positionTarget, position, velocity, 32000);
    double force = SimmyAnalogChainStep(&analogState, &analogParams, (double)torque);

    int32_t err = Abs32(positionTarget - position);
    if (i < accelTicks) { if (err > maxErrAccel) maxErrAccel = err; }
    else { if (err > maxErrCruise) maxErrCruise = err; }
    if (Abs32(torque) >= gains.torqueClampMax - 1) pegTicks++;

    SimmyChainStep(&state, &params, force);
  }

  printf("maxErrAccel=%d maxErrCruise=%d pegTicks=%d (of %d ticks)\n",
         maxErrAccel, maxErrCruise, pegTicks, ticks);
  printf("analog-servo's own calibrated bounds for this scenario "
         "(test_prototype_nema23_leadscrew_preset_jittered_is_stable): "
         "maxErrAccel<=450, maxErrCruise<=300, pegTicks<=20\n");

  int ok = maxErrAccel <= 450 && maxErrCruise <= 300 && pegTicks <= 20;
  printf("%s\n", ok ? "PASS -- real Control.c, linked via simmy, reproduces analog-servo's "
                      "own validated behavior" : "FAIL -- investigate before trusting this linkage");
  return ok ? 0 : 1;
}
