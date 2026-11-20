#include "../examples/stepper_experiment.h"
#include <assert.h>

static StepperCompensation policy(int ff, int damping) {
  StepperCompensation c = {.detentEstimate=ff ? .012 : 0, .detentPhaseEstimate=.3,
      .detentCyclesEstimate=200, .damping=damping ? .08 : 0,
      .torqueLimit=.2, .phaseCurrentLimit=1.5};
  return c;
}

static StepperMetrics trial(int scenario, double speed, int hz, int ff, int damping) {
  StepperCompensation c = policy(ff, damping);
  StepperMetrics m = StepperExperimentControlled(&stepperScenarios[scenario], speed, hz,
                                                2, StepperSweepWindow(speed), NULL, &c);
  assert(isfinite(m.rmsError));
  assert(m.peakPhaseCurrent <= c.phaseCurrentLimit);
  assert(m.correctionRms <= c.torqueLimit);
  assert(m.torqueLimitFraction == 0 && m.currentLimitFraction == 0);
  assert(m.maxAbsLoadAngle < STEPPER_PI);
  return m;
}

int main(void) {
  /* Zero correction must preserve the original experiment, including its
   * imperfect drive. No hidden direct-force actuation or current bypass. */
  for (size_t i = 0; i < STEPPER_SCENARIOS; i++) {
    StepperMetrics old = StepperExperiment(&stepperScenarios[i], .2, 10000, 2, 2, NULL);
    StepperCompensation c = policy(0, 0);
    StepperMetrics same = StepperExperimentControlled(&stepperScenarios[i], .2, 10000, 2, 2, NULL, &c);
    assert(fabs(old.rmsError-same.rmsError) < 1e-13);
    assert(fabs(old.currentVectorRms-same.currentVectorRms) < 1e-13);
  }
  StepperMetrics detent = trial(3, .2, 10000, 0, 0);
  StepperMetrics ff = trial(3, .2, 10000, 1, 0);
  assert(ff.rmsError < detent.rmsError/100);
  /* Damping suppresses resonance, but does not interpolate the step
   * command or eliminate the staircase's excitation. */
  StepperMetrics resonance = trial(1, .0698, 10000, 0, 0);
  StepperMetrics damped = trial(1, .0698, 10000, 0, 1);
  assert(damped.rmsError < resonance.rmsError/5);
  assert(damped.rmsError > .05);
  StepperMetrics combined = trial(5, .2, 10000, 0, 0);
  StepperMetrics both = trial(5, .2, 10000, 1, 1);
  assert(both.rmsError < combined.rmsError/2);
  StepperMetrics fine = trial(5, .2, 20000, 1, 1);
  assert(fabs(fine.rmsError/both.rmsError-1) < .02);
  /* A wrong table is not magically corrected by access to plant detent. */
  StepperCompensation wrong = policy(1, 0);
  wrong.detentPhaseEstimate += STEPPER_PI;
  StepperMetrics wrongTable = StepperExperimentControlled(&stepperScenarios[3], .2, 10000,
                                             2, StepperSweepWindow(.2), NULL, &wrong);
  assert(wrongTable.rmsError > detent.rmsError*1.5);
  /* Explicitly exercise both actuator limits, not only the unsaturated
   * default experiment. No claim that this reduced-current case tracks. */
  StepperCompensation limited = policy(1, 1);
  limited.torqueLimit = .001;
  limited.phaseCurrentLimit = .5;
  StepperMetrics clipped = StepperExperimentControlled(&stepperScenarios[5], .2, 10000,
                                                       .1, .2, NULL, &limited);
  assert(clipped.torqueLimitFraction > 0);
  assert(clipped.currentLimitFraction > 0);
  assert(clipped.correctionRms <= .001 + 1e-15);
  assert(clipped.peakPhaseCurrent <= .5);
  puts("Stepper compensation tests passed.");
  return 0;
}
