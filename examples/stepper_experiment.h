#ifndef STEPPER_EXPERIMENT_H
#define STEPPER_EXPERIMENT_H

/* Shared experiment fixture, not a library API. SI parameters and initialization
 * are deliberately identical for trace and sweep experiments. */
#include "simmy.h"
#include <math.h>
#include <stdio.h>

#define STEPPER_PI 3.14159265358979323846
#define STEPPER_COUNTS_PER_RADIAN 10000.0

typedef struct {
  const char *name;
  int microsteps, detent, errors;
} StepperScenario;

static const StepperScenario stepperScenarios[] = {
  {"continuous ideal", 0, 0, 0}, {"16 microsteps", 16, 0, 0},
  {"64 microsteps", 64, 0, 0}, {"detent only", 0, 1, 0},
  {"current errors only", 0, 0, 1}, {"combined 16", 16, 1, 1}
};
#define STEPPER_SCENARIOS (sizeof(stepperScenarios)/sizeof(stepperScenarios[0]))

typedef struct {
  double meanSpeed, rmsError, acRms, peakToPeak;
  double maxAbsLoadAngle; /* UNWRAPPED; over settling AND measurement */
  double finalLoadAngle, reverseFraction;
  int samples;
} StepperMetrics;

/* Speed positive, hz>0, settling>=0, measurement>0. trace=NULL suppresses
 * time-series output. Each run starts at ideal loaded traveling equilibrium;
 * this tests sustained tracking, NOT acceleration/pull-in from rest. */
static StepperMetrics StepperExperiment(const StepperScenario *scenario, double target,
                                       int hz, double settling, double measurement, FILE *trace) {
  SimmyStepperParams motor = {.electricalCyclesPerRev=50, .torqueConstant=.4,
      .detentAmplitude=scenario->detent ? .012 : 0, .detentCyclesPerRev=200, .detentPhase=.3};
  SimmyStepperCurrentParams path = {0};
  if (scenario->errors)
    path = (SimmyStepperCurrentParams){.gainError={.03,-.03}, .offset={.01,-.015},
                                       .quantizeStep={1.0/31,1.0/31}};
  /* J=4e-4 kg m^2 (rotor + reflected load), B=.008 Nm/(rad/s), load=.03 Nm.
   * x=C*theta; tick=1/hz s; force unit=Nm.
   * inertia=J*hz^2/C, viscous=B*hz/C. No body ripple (detent is in motor). */
  SimmyChainParams p = {.bodyCount=1, .subSteps=20};
  p.bodies[0].inertia = 4e-4 * hz * hz / STEPPER_COUNTS_PER_RADIAN;
  p.bodies[0].friction = .008 * hz / STEPPER_COUNTS_PER_RADIAN;
  p.bodies[0].gravityBias = .03;
  SimmyChainState state;
  SimmyChainInit(&state, &p);
  state.bodies[0].position = -asin((.03+.008*target)/.4)/50 * STEPPER_COUNTS_PER_RADIAN;
  state.bodies[0].velocity = target * STEPPER_COUNTS_PER_RADIAN / hz;
  SimmyPulseTrainState pulse;
  SimmyPulseTrainInit(&pulse, 1);
  int64_t index = 0;
  int warmTicks = (int)ceil(settling*hz), samples = (int)ceil(measurement*hz);
  StepperMetrics result = {.samples=samples};
  double sum = 0, sumErrorSquared = 0, m2 = 0, mean = 0;
  double minimum = INFINITY, maximum = -INFINITY;
  int reversed = 0, count = 0;
  for (int tick = 0; tick < warmTicks+samples; tick++) {
    double time = (double)tick/hz;
    double unwrappedPhi = scenario->microsteps
        ? (double)index*STEPPER_PI/(2*scenario->microsteps) : 50*target*time;
    double phi = scenario->microsteps
        ? SimmyStepperMicrostepAngle(index, scenario->microsteps) : unwrappedPhi;
    double currents[2];
    SimmyStepperCurrents(phi, 1, &path, currents);
    double theta = state.bodies[0].position/STEPPER_COUNTS_PER_RADIAN;
    double speed = state.bodies[0].velocity*hz/STEPPER_COUNTS_PER_RADIAN;
    double loadAngle = unwrappedPhi-50*theta;
    result.maxAbsLoadAngle = fmax(result.maxAbsLoadAngle, fabs(loadAngle));
    result.finalLoadAngle = loadAngle;
    if (tick >= warmTicks) {
      if (trace)
        fprintf(trace, "%s,%.7f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", scenario->name, time,
                theta, speed, remainder(loadAngle, 2*STEPPER_PI), currents[0], currents[1],
                SimmyStepperTorque(&motor, theta, currents[0], currents[1]));
      sum += speed;
      sumErrorSquared += (speed-target)*(speed-target);
      /* Welford variance avoids cancellation for the almost-ideal case. */
      double delta = speed-mean;
      mean += delta/++count;
      m2 += delta*(speed-mean);
      minimum = fmin(minimum, speed);
      maximum = fmax(maximum, speed);
      reversed += speed < 0;
    }
    SimmyStepperChainStep(&state, &p, &motor, 1/STEPPER_COUNTS_PER_RADIAN, 1, currents[0], currents[1]);
    if (scenario->microsteps)
      index += SimmyPulseTrainTickDelta(&pulse, 50*target*2*scenario->microsteps/STEPPER_PI, hz, 0);
  }
  result.meanSpeed = sum/samples;
  result.rmsError = sqrt(sumErrorSquared/samples);
  result.acRms = sqrt(fmax(0, m2/samples));
  result.peakToPeak = maximum-minimum;
  result.reverseFraction = (double)reversed/samples;
  return result;
}

/* At least 2 seconds AND four complete commanded electrical cycles.
 * Integer cycles reduce endpoint bias; actual elapsed ticks round upward. */
static double StepperSweepWindow(double speed) {
  double period = 2*STEPPER_PI/(50*speed);
  return fmax(4, ceil(2/period))*period;
}
#endif
