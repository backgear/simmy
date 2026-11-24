#include "../examples/voicecoil_pid_experiment.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static void near(double a, double b, double tolerance) { assert(fabs(a - b) < tolerance); }

/* Both step commands settle within a tight tolerance -- confirms the PID
 * (not just the open-loop plant) actually reaches and holds each target,
 * including the "up" target where gravityBias must be continuously
 * cancelled (see hold-current check below) and the "down" target, which
 * only gravity would also reach, but NOT in a controlled way (see
 * descent_is_controlled_not_free_fall() below). */
static void both_steps_settle(void) {
  VoicecoilPidRig rig;
  VoicecoilPidRigInit(&rig);
  const int totalTicks = 60000; /* 6s: matches examples/voicecoil_pid.c's schedule */
  for (int tick = 0; tick < totalTicks; tick++) {
    double time = tick * rig.mount.tickSeconds;
    VoicecoilPidTick(&rig, VoicecoilPidTarget(time));
    if (tick == 24999) { /* t=2.5s, just before the down step: "up" settled */
      int32_t position = SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
      assert(abs(position - 2000) < 30);
      /* The integral has learned to cancel gravity: holding current should
       * be essentially exactly gravityBias/forceConstant, not left sagging
       * under a P-only steady-state error. */
      near(rig.electrical.current, rig.chain.bodies[0].gravityBias / rig.motor.forceConstant, .02);
    }
  }
  int32_t finalPosition = SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
  assert(abs(finalPosition - 0) < 30); /* "down" settled back at the bottom target */
  near(rig.electrical.current, rig.chain.bodies[0].gravityBias / rig.motor.forceConstant, .02);
  puts("Voice-coil PID: both the lift and the descend-back-down steps settle near target, "
       "with the integral term learning to cancel gravityBias (no steady-state sag).");
}

/* The point of this example: a passive (zero-voltage) actuator under
 * gravityBias alone does NOT stop at the down target -- it accelerates
 * past it and keeps falling (no mechanical stop is modeled; see
 * docs/scope.md's "Endstops and hard limits"). The PID must therefore be
 * actively braking/reversing on the way down, not just releasing current
 * and letting gravity carry it to the target. Compare identical starting
 * states (the "up"-settled rig, cloned) under PID control vs. open-loop
 * zero voltage over the same short window right after the down command. */
static void descent_is_controlled_not_free_fall(void) {
  VoicecoilPidRig rig;
  VoicecoilPidRigInit(&rig);
  for (int tick = 0; tick < 25000; tick++) { /* settle at the "up" target, t=0..2.5s */
    VoicecoilPidTick(&rig, VoicecoilPidTarget(tick * rig.mount.tickSeconds));
  }
  int32_t settledPosition = SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
  assert(abs(settledPosition - 2000) < 30);

  VoicecoilPidRig freeFall = rig; /* identical electrical/mechanical state at the moment of the down step */

  const int windowTicks = 2000; /* 0.2s */
  for (int tick = 0; tick < windowTicks; tick++) {
    VoicecoilPidTick(&rig, 0); /* PID commanded down */
    SimmyVoiceCoilChainStep(&freeFall.mechanics, &freeFall.chain, &freeFall.electrical,
                             &freeFall.motor, &freeFall.mount, 0, freeFall.mount.tickSeconds); /* 0V: gravity only */
  }

  int32_t pidPosition = SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
  int32_t freeFallPosition = SimmyChainFeedbackPositionCounts(&freeFall.mechanics, &freeFall.chain);
  /* PID stays close to the target it's heading for; free fall has already
   * overshot well past it and is still accelerating away, with nothing to
   * stop it -- the contrast this test exists to show. */
  assert(abs(pidPosition) < 150);
  assert(freeFallPosition < -200);
  assert(freeFall.mechanics.bodies[0].velocity < rig.mechanics.bodies[0].velocity - 0.1);
  printf("Voice-coil PID: 0.2s after the down command, PID position %d counts vs. free-fall %d counts "
         "-- descent is actively controlled, not released to gravity.\n", pidPosition, freeFallPosition);
}

int main(void) {
  both_steps_settle();
  descent_is_controlled_not_free_fall();
  puts("Voice-coil vertical PID example tests passed.");
  return 0;
}
