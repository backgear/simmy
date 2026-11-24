/* Two linear electromagnetic actuators driving a single mechanical body
 * each -- the same one-body-chain pattern dc_motor.c uses for the
 * brushed DC motor, just with SimmyLinearMount in place of the rotary
 * SimmyMotorMount.
 *
 * Phase 1, "reluctance_pullin": the iron-core sample reluctance motor
 * (see docs/motors/solenoid.md) held at a constant 6V. Starting 3mm from
 * the mechanical stop, it pulls itself in -- and ONLY in, since
 * SimmyReluctanceOutput.force is always <= 0 in this model's gap
 * convention (see include/simmy_solenoid.h).
 *
 * Phase 2, "voicecoil_pushpull": the sample voice-coil motor driven by a
 * 0.5 Hz, 8V sine wave. Unlike phase 1, this motor pushes AND pulls --
 * velocity tracks the sign of the drive voltage, which a reluctance
 * solenoid structurally cannot do.
 *
 * Mechanical loads are deliberately heavier/more damped than dc_motor.c's
 * -- the body chain's mechanical integrator is plain explicit Euler, and
 * both motors' forces are strong enough at these voltages that a light
 * load needs finer substepping than this example cares to exercise.
 * That is a property of the simple mechanical integrator, not of either
 * motor's own (implicit, unconditionally stable) electrical step.
 */
#include "simmy.h"
#include <math.h>
#include <stdio.h>

static void RunReluctance(void) {
  /* Sample "iron-core" reluctance motor -- see docs/motors/solenoid.md. */
  SimmyReluctanceParams motor = {
      .resistance = 2.0, .minInductance = .0005, .maxInductance = .05,
      .shapeMeters = .0005, .saturationSharpness = 2.0, .minGapMeters = .0002};
  SimmyReluctanceState electrical;
  SimmyReluctanceInit(&electrical);

  SimmyLinearMount mount = {.metersPerCount = 1e-6, .forcePerNewton = 1, .tickSeconds = .0001};
  SimmyChainParams chain = {0};
  chain.bodyCount = 1;
  chain.subSteps = 1;
  chain.bodies[0].inertia = 2000;
  chain.bodies[0].friction = 100;
  SimmyChainState mechanics;
  SimmyChainInit(&mechanics, &chain);
  mechanics.bodies[0].position = 3000; /* 3mm starting gap at 1e-6 m/count */

  const double voltage = 6.0;
  const int totalTicks = 31000; /* 3.1s -- well into the pull-in, short of the mechanical stop:
                                    this example doesn't model a hard stop on the body chain
                                    itself (see include/simmy_solenoid.h), so running long
                                    enough to reach minGapMeters would run the gap straight
                                    through zero and negative. */

  for (int tick = 0; tick < totalTicks; tick++) {
    double t = tick * mount.tickSeconds;
    if (!SimmyReluctanceChainStep(&mechanics, &chain, &electrical, &motor, &mount,
                                   voltage, mount.tickSeconds)) {
      fprintf(stderr, "SimmyReluctanceChainStep: invalid state/parameters\n");
      continue;
    }
    double gap = mechanics.bodies[0].position * mount.metersPerCount;
    double velocity = mechanics.bodies[0].velocity * mount.metersPerCount / mount.tickSeconds;
    printf("reluctance_pullin,%d,%.6f,%.3f,%.6f,%.9f,%.9f\n", tick, t, voltage,
           electrical.current, gap, velocity);
  }
}

static void RunVoiceCoil(void) {
  /* Sample voice-coil motor -- see docs/motors/solenoid.md. */
  SimmyVoiceCoilParams motor = {.resistance = 3.0, .inductance = .0006, .forceConstant = 1.2};
  SimmyVoiceCoilState electrical;
  SimmyVoiceCoilInit(&electrical);

  SimmyLinearMount mount = {.metersPerCount = 1e-6, .forcePerNewton = 1, .tickSeconds = .0001};
  SimmyChainParams chain = {0};
  chain.bodyCount = 1;
  chain.subSteps = 1;
  chain.bodies[0].inertia = 20;
  chain.bodies[0].friction = 1;
  SimmyChainState mechanics;
  SimmyChainInit(&mechanics, &chain);

  const double amplitudeV = 8.0;
  const double frequencyHz = .5;
  const int totalTicks = 40000; /* 4s -- two full drive cycles */

  for (int tick = 0; tick < totalTicks; tick++) {
    double t = tick * mount.tickSeconds;
    double voltage = amplitudeV * sin(2 * M_PI * frequencyHz * t);
    if (!SimmyVoiceCoilChainStep(&mechanics, &chain, &electrical, &motor, &mount,
                                  voltage, mount.tickSeconds)) {
      fprintf(stderr, "SimmyVoiceCoilChainStep: invalid state/parameters\n");
      continue;
    }
    double position = mechanics.bodies[0].position * mount.metersPerCount;
    double velocity = mechanics.bodies[0].velocity * mount.metersPerCount / mount.tickSeconds;
    printf("voicecoil_pushpull,%d,%.6f,%.6f,%.6f,%.9f,%.9f\n", tick, t, voltage,
           electrical.current, position, velocity);
  }
}

int main(void) {
  printf("family,tick,time,voltage,current,position,velocity\n");
  RunReluctance();
  RunVoiceCoil();
  return 0;
}
