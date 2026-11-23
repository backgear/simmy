/* Small open-loop runs under a reversing "ping-pong" force, each isolating
 * one mechanical-chain effect so its signature is easy to recognize later
 * in noisier real data. No controller: force is applied directly, every
 * tick, with nothing closing a loop around position or velocity.
 *
 * - inertia_only:     body.friction = stictionLimit = coulombLevel = 0.
 * - viscous_friction:  + body.friction.
 * - stiction:          + stictionLimit/coulombLevel instead (friction = 0,
 *                      to keep the breakaway plateau from being blurred by
 *                      viscous drag), under a RAMPED force -- a step bigger
 *                      than stictionLimit breaks away the same tick, and a
 *                      step smaller never breaks away at all, so neither
 *                      shows the plateau. A slow ramp crossing the
 *                      threshold is what actually reveals it, which is
 *                      exactly why real breakaway tests use slow traverses
 *                      rather than step commands.
 * - backlash:          a 2-body chain, same step force on body 0 (motor);
 *                      body 1 (load) only feels it through a backlash
 *                      coupling.
 */
#include "simmy.h"
#include <stdio.h>

/* A true reversing (not just "accelerate then brake to a stop") square
 * wave: starting from rest, a half-length first pulse phases the sign
 * flips so the body settles into a genuine back-and-forth oscillation
 * instead of a one-way staircase that merely touches zero velocity at
 * each boundary. */
static double SquareWave(int tick, int halfPeriodTicks, double amplitude) {
  int quarter = halfPeriodTicks / 2;
  if (tick < quarter) return amplitude;
  int t = tick - quarter;
  return ((t / halfPeriodTicks) % 2 == 0) ? -amplitude : amplitude;
}

/* A continuous triangle wave in force, starting at 0 and ramping up (not
 * starting at a peak), so a breakaway threshold gets crossed gradually --
 * from an actual stop -- instead of jumped over. */
static double TriangleWave(int tick, int periodTicks, double amplitude) {
  double phase = (double)(tick % periodTicks) / (double)periodTicks;
  double triangle;
  if (phase < .25) triangle = 4.0 * phase;
  else if (phase < .75) triangle = 2.0 - 4.0 * phase;
  else triangle = 4.0 * phase - 4.0;
  return amplitude * triangle;
}

typedef double (*ForceWave)(int tick, int periodTicks, double amplitude);

static void RunSingleBody(const char *scenario, double friction, double stictionLimit,
                          double coulombLevel, int ticks, int periodTicks, double amplitude,
                          ForceWave wave) {
  SimmyChainParams params = {0};
  params.bodyCount = 1;
  params.subSteps = 1;
  params.bodies[0].inertia = 50;
  params.bodies[0].friction = friction;
  params.bodies[0].stictionLimit = stictionLimit;
  params.bodies[0].coulombLevel = coulombLevel;
  SimmyChainState state;
  SimmyChainInit(&state, &params);

  for (int tick = 0; tick < ticks; tick++) {
    double force = wave(tick, periodTicks, amplitude);
    printf("%s,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n", scenario, tick, force,
           state.bodies[0].position, state.bodies[0].velocity,
           state.bodies[0].position, state.bodies[0].velocity);
    SimmyChainStep(&state, &params, force);
  }
}

static void RunBacklash(int ticks, int halfPeriodTicks, double amplitude) {
  SimmyChainParams params = {0};
  params.bodyCount = 2;
  params.subSteps = 20;
  params.bodies[0].inertia = 50;
  params.bodies[1].inertia = 50;
  params.couplings[0].springK = 40;
  params.couplings[0].springD = 4;
  params.couplings[0].backlashGap = 8;
  SimmyChainState state;
  SimmyChainInit(&state, &params);

  for (int tick = 0; tick < ticks; tick++) {
    double force = SquareWave(tick, halfPeriodTicks, amplitude);
    printf("backlash,%d,%.6f,%.6f,%.6f,%.6f,%.6f\n", tick, force,
           state.bodies[0].position, state.bodies[0].velocity,
           state.bodies[1].position, state.bodies[1].velocity);
    SimmyChainStep(&state, &params, force);
  }
}

int main(void) {
  const int ticks = 6000, halfPeriod = 1500;
  const double amplitude = 2.0;
  printf("scenario,tick,force,motor_position,motor_velocity,load_position,load_velocity\n");
  RunSingleBody("inertia_only", 0, 0, 0, ticks, halfPeriod, amplitude, SquareWave);
  RunSingleBody("viscous_friction", .3, 0, 0, ticks, halfPeriod, amplitude, SquareWave);
  RunSingleBody("stiction", 0, 1.0, .6, ticks, 3000, 3.0, TriangleWave);
  RunBacklash(ticks, halfPeriod, amplitude);
  return 0;
}
