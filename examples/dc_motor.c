/* A brushed DC motor driving a single mechanical body -- the simplest of
 * simmy's four motor models: one winding, Kirchhoff's voltage law, no
 * commutation and no drive clock of its own. The electrical step and the
 * mechanical tick run at the same rate; there is nothing else to schedule.
 *
 * Constant 12V drive, no speed controller. A passive Coulomb/static brake
 * engages at 1s to show loaded vs. unloaded steady speed and current.
 */
#include "simmy.h"
#include <stdio.h>

int main(void) {
  SimmyDcParams motor = {.resistance = .6, .inductance = .002, .backEmfConstant = .08};
  SimmyDcState electrical;
  SimmyDcInit(&electrical);

  SimmyMotorMount mount = {.radiansPerCount = .001, .forcePerNm = 1, .tickSeconds = .0001};
  SimmyChainParams chain = {0};
  chain.bodyCount = 1;
  chain.subSteps = 1;
  /* J=.0008 kg m^2, B=.0005 Nm/(rad/s) -- same scale as the BLDC sample motor. */
  chain.bodies[0].inertia = .0008 / (1000 * .0001 * .0001);
  chain.bodies[0].friction = .0005 / (1000 * .0001);
  SimmyChainState mechanics;
  SimmyChainInit(&mechanics, &chain);

  const double voltage = 12.0;
  const double brakeStartSeconds = 1.0;
  const double brakeNm = .05;
  const int totalTicks = 20000; /* 2 seconds at 10 kHz */

  printf("tick,time,voltage,current,torque,speed,position\n");
  for (int tick = 0; tick < totalTicks; tick++) {
    double t = tick * mount.tickSeconds;
    if (t >= brakeStartSeconds) {
      chain.bodies[0].coulombLevel = brakeNm;
      chain.bodies[0].stictionLimit = brakeNm;
    }

    double speed = mechanics.bodies[0].velocity * mount.radiansPerCount / mount.tickSeconds;
    SimmyDcOutput output;
    if (!SimmyDcStep(&electrical, &motor, voltage, speed, mount.tickSeconds, &output)) {
      fprintf(stderr, "SimmyDcStep: invalid state/parameters\n");
      return 1;
    }
    SimmyChainStepDt(&mechanics, &chain, output.torque / mount.forcePerNm, 1.0);
    double position = mechanics.bodies[0].position * mount.radiansPerCount;

    printf("%d,%.6f,%.3f,%.6f,%.6f,%.6f,%.6f\n", tick, t, voltage, electrical.current,
           output.torque, speed, position);
  }
  return 0;
}
