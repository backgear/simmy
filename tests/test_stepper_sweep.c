/* Regression tests for the shared speed-sweep experiment and metrics. */
#include "../examples/stepper_experiment.h"
#include <assert.h>

static StepperMetrics trial(int scenario, double speed, int hz) {
  StepperMetrics m = StepperExperiment(&stepperScenarios[scenario], speed, hz,
                                     2, StepperSweepWindow(speed), NULL);
  assert(isfinite(m.rmsError) && isfinite(m.acRms));
  assert(m.samples >= 2*hz);
  /* RMS includes both DC tracking error and AC velocity ripple. */
  double bias = m.meanSpeed-speed;
  assert(fabs(m.rmsError*m.rmsError - m.acRms*m.acRms - bias*bias) < 1e-11);
  assert(fabs(bias/speed) < .001);
  assert(m.maxAbsLoadAngle < STEPPER_PI);
  return m;
}

int main(void) {
  double period = 2*STEPPER_PI/(50*.01);
  assert(fabs(StepperSweepWindow(.01)/period-4) < 1e-14);
  assert(StepperSweepWindow(1.2) >= 2);
  double cycles = StepperSweepWindow(.2)/(2*STEPPER_PI/(50*.2));
  assert(fabs(cycles-round(cycles)) < 1e-14);

  StepperMetrics ideal = trial(0, .2, 10000);
  assert(ideal.rmsError/.2 < .001 && ideal.reverseFraction == 0);
  StepperMetrics original = StepperExperiment(&stepperScenarios[1], .2, 10000, 2, 2, NULL);
  assert(fabs(original.rmsError/.2 - .204507) < .0001);

  /* Fundamental microstep-update resonance moves when M changes. */
  StepperMetrics m64 = trial(2, .017481, 10000);
  StepperMetrics m64off = trial(2, .04, 10000);
  assert(m64.rmsError > 3*m64off.rmsError);
  assert(m64.reverseFraction > .4);
  StepperMetrics m16 = trial(1, .0698, 10000);
  StepperMetrics m16off = trial(1, .2, 10000);
  assert(m16.rmsError > 10*m16off.rmsError);
  StepperMetrics detent = trial(3, 1.107971, 10000);
  StepperMetrics detentOff = trial(3, .5, 10000);
  assert(detent.rmsError > 5*detentOff.rmsError);

  /* Halving dt preserves the physical peaks (not a claim that tiny
   * off-resonance pulse-timing artifacts have converged everywhere). */
  StepperMetrics m64fine = trial(2, .017481, 20000);
  StepperMetrics m16fine = trial(1, .0698, 20000);
  StepperMetrics detentFine = trial(3, 1.107971, 20000);
  assert(fabs(m64fine.rmsError/m64.rmsError - 1) < .005);
  assert(fabs(m16fine.rmsError/m16.rmsError - 1) < .005);
  assert(fabs(detentFine.rmsError/detent.rmsError - 1) < .005);
  puts("Stepper sweep tests passed.");
  return 0;
}
