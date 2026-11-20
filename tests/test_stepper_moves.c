/* Abrupt starts and energized stops: demonstrate actual loss of synchronism,
 * not merely a wrapped-angle warning. Optional plotting data:
 *   build/test_stepper_moves --csv build/stepper_moves.csv
 * The deliberately fast cases test this ideal-current model's dynamics,
 * NOT a real motor's high-speed pull-in/pull-out curve. */
#include "simmy.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846
#define C 10000.0
#define N 50
#define M 16

static const SimmyStepperParams motor = {.electricalCyclesPerRev=N, .torqueConstant=.4};

typedef struct {
  SimmyChainParams params;
  SimmyChainState state;
  int hz, ticks;
  int64_t index;
  double maxMoveLag, stopPosition, stopVelocity, maxStopPosition, minStopPosition;
  int stopReversed;
  const char *name;
  FILE *trace;
} Trial;

static double angle(const Trial *t) { return t->state.bodies[0].position/C; }
static double velocity(const Trial *t) { return t->state.bodies[0].velocity*t->hz/C; }
static double command(const Trial *t) { return (double)t->index*PI/(2*M*N); }
static double errorCycles(const Trial *t) { return (angle(t)-command(t))*N/(2*PI); }

static Trial init(const char *name, int hz, FILE *trace) {
  Trial t = {.hz=hz, .name=name, .trace=trace};
  t.params.bodyCount = 1;
  t.params.subSteps = 20;
  t.params.bodies[0].inertia = 4e-4*hz*hz/C;
  t.params.bodies[0].friction = .008*hz/C;
  /* No load, detent, or current error: isolate magnetic pull-in/capture. */
  SimmyChainInit(&t.state, &t.params);
  return t;
}

static void advance(Trial *t, const char *phase) {
  if (t->trace && t->ticks % (t->hz/1000) == 0)
    fprintf(t->trace, "%s,%d,%.7f,%s,%lld,%.9f,%.9f,%.9f,%.9f,%.9f\n",
            t->name, t->hz, (double)t->ticks/t->hz, phase, (long long)t->index,
            command(t), angle(t), velocity(t), (command(t)-angle(t))*N, errorCycles(t));
  double phi = SimmyStepperMicrostepAngle(t->index, M);
  SimmyStepperChainStep(&t->state, &t->params, &motor, 1/C, 1, cos(phi), sin(phi));
  t->ticks++;
  assert(isfinite(angle(t)) && isfinite(velocity(t)));
}

/* Stop means freeze the signed command index, NOT zero rotor velocity or
 * disable the driver. The final phase currents remain energized for 2 s. */
static void hold(Trial *t, int direction) {
  int64_t finalIndex = t->index;
  t->stopPosition = angle(t);
  t->stopVelocity = velocity(t);
  t->maxStopPosition = t->minStopPosition = angle(t);
  for (int i = 0; i < 2*t->hz; i++) {
    advance(t, "hold");
    assert(t->index == finalIndex);
    t->maxStopPosition = fmax(t->maxStopPosition, angle(t));
    t->minStopPosition = fmin(t->minStopPosition, angle(t));
    t->stopReversed |= direction*velocity(t) < -1e-5;
  }
  assert(fabs(velocity(t)) < 1e-6);
  /* Ending at a stable magnetic alignment does not imply the correct
   * unwrapped position: all integer-cycle offsets have the same currents. */
  assert(fabs(errorCycles(t)-round(errorCycles(t))) < 1e-6);
}

static Trial move(const char *name, int hz, double speed, int pulses, FILE *trace) {
  Trial t = init(name, hz, trace);
  SimmyPulseTrainState source;
  SimmyPulseTrainInit(&source, 1);
  int direction = speed > 0 ? 1 : -1;
  assert(angle(&t) == 0 && velocity(&t) == 0);
  while (direction*t.index < pulses) {
    int32_t delta = SimmyPulseTrainTickDelta(&source, speed*N*2*M/PI, hz, 0);
    int64_t remaining = pulses-direction*t.index;
    if (direction*(int64_t)delta > remaining) delta = (int32_t)(direction*remaining);
    t.index += delta;
    advance(&t, "move");
    t.maxMoveLag = fmax(t.maxMoveLag, fabs((command(&t)-angle(&t))*N));
    assert(t.ticks < 5*hz);
  }
  assert(t.index == direction*(int64_t)pulses); /* exact finite pulse count */
  hold(&t, direction);
  printf("%s (%d Hz): startup max lag %.3f cycles; settled error %+.0f cycles (%+.0f full steps)\n",
         name, hz, t.maxMoveLag/(2*PI), errorCycles(&t), 4*errorCycles(&t));
  return t;
}

/* Isolate stopping from startup: begin on the ideal traveling solution at
 * phi=0, then freeze phi immediately. No startup error exists to inherit. */
static Trial stop(const char *name, int hz, double speed, FILE *trace) {
  Trial t = init(name, hz, trace);
  t.state.bodies[0].position = -asin(.008*speed/.4)/N*C;
  t.state.bodies[0].velocity = speed*C/hz;
  assert(fabs(errorCycles(&t)) < .1);
  hold(&t, speed > 0 ? 1 : -1);
  printf("%s (%d Hz): incoming speed %.1f rad/s; settled error %+.0f cycles (%+.0f full steps)\n",
         name, hz, speed, errorCycles(&t), 4*errorCycles(&t));
  return t;
}

int main(int argc, char **argv) {
  FILE *trace = NULL;
  if (argc == 3 && !strcmp(argv[1], "--csv")) {
    trace = fopen(argv[2], "w");
    if (!trace) { perror(argv[2]); return 1; }
    fputs("scenario,tick_hz,time_s,phase,microstep_index,command_rad,position_rad,velocity_rad_s,"
          "load_angle_rad,error_cycles\n", trace);
  } else if (argc != 1) {
    fprintf(stderr, "Usage: %s [--csv PATH]\n", argv[0]);
    return 1;
  }
  for (int direction = -1; direction <= 1; direction += 2) {
    Trial low = move(direction > 0 ? "slow start forward" : "slow start reverse",
                     10000, direction*.2, 128, trace);
    assert(low.maxMoveLag < PI); /* stays near the intended alignment */
    assert(fabs(errorCycles(&low)) < 1e-6);
    assert(low.stopReversed); /* energized capture rings, not a velocity snap */

    Trial fast = move(direction > 0 ? "fast start forward" : "fast start reverse",
                      10000, direction*20, 1024, trace);
    assert(fast.maxMoveLag > 4*PI); /* several actual field revolutions lost */
    assert(direction*errorCycles(&fast) < -2); /* relocks BEHIND command */

    Trial gentle = stop(direction > 0 ? "gentle stop forward" : "gentle stop reverse",
                        10000, direction*.2, trace);
    assert(fabs(errorCycles(&gentle)) < 1e-6);
    assert(gentle.stopReversed);
    assert(direction > 0 ? gentle.maxStopPosition > 0 : gentle.minStopPosition < 0);

    Trial hard = stop(direction > 0 ? "hard stop forward" : "hard stop reverse",
                      10000, direction*20, trace);
    assert(direction*errorCycles(&hard) >= 1); /* relocks AHEAD of frozen field */
    assert(fabs(velocity(&hard)) < fabs(hard.stopVelocity)/1000);

    /* Halved outer dt preserves the discrete slipped-well outcomes, not
     * just wrapped alignment. Integration still uses 20 substeps/tick. */
    Trial fastFine = move(direction > 0 ? "fast start forward fine" : "fast start reverse fine",
                          20000, direction*20, 1024, trace);
    Trial hardFine = stop(direction > 0 ? "hard stop forward fine" : "hard stop reverse fine",
                         20000, direction*20, trace);
    assert(round(errorCycles(&fastFine)) == round(errorCycles(&fast)));
    assert(round(errorCycles(&hardFine)) == round(errorCycles(&hard)));
  }
  if (trace) {
    if (fclose(trace)) { perror("closing trace"); return 1; }
  }
  puts("Stepper start/stop slip tests passed.");
  return 0;
}
