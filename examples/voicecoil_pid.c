/* Closed-loop position control of a vertical voice-coil actuator against
 * gravity -- see voicecoil_pid_experiment.h for the plant, controller,
 * and why both are shaped the way they are (genuine PID with
 * back-calculation anti-windup, voltage-mode drive, no mechanical return
 * spring).
 *
 * Schedule: hold at the bottom (target 0) until 0.5s, step up to 2000
 * counts (2mm at this example's metersPerCount) until 2.5s, then step
 * back down to 0 -- demonstrating the controller actively brakes/reverses
 * on the way down rather than just releasing current and free-falling
 * (see tests/test_voicecoil_pid.c for a quantitative check of that).
 *
 * Run from the repo root: ./build/voicecoil_pid
 */
#include "voicecoil_pid_experiment.h"
#include <stdio.h>

int main(void) {
  VoicecoilPidRig rig;
  VoicecoilPidRigInit(&rig);

  const int totalTicks = 60000; /* 6s */

  printf("tick,time,target,position,velocity,voltage,current\n");
  for (int tick = 0; tick < totalTicks; tick++) {
    double time = tick * rig.mount.tickSeconds;
    double target = VoicecoilPidTarget(time);
    double voltage = VoicecoilPidTick(&rig, target);
    int32_t position = SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
    int32_t velocity = SimmyChainFeedbackVelocityCounts(&rig.mechanics, &rig.chain);
    printf("%d,%.6f,%.1f,%d,%d,%.6f,%.6f\n", tick, time, target, position, velocity,
           voltage, rig.electrical.current);
  }

  int32_t finalError = (int32_t)VoicecoilPidTarget(totalTicks * rig.mount.tickSeconds) -
      SimmyChainFeedbackPositionCounts(&rig.mechanics, &rig.chain);
  fprintf(stderr, "final following error: %d counts; hold current ~%.3fA (gravityBias/forceConstant = %.3fA)\n",
          finalError, rig.electrical.current, rig.chain.bodies[0].gravityBias / rig.motor.forceConstant);
  return 0;
}
