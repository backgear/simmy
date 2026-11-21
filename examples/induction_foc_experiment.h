#ifndef INDUCTION_FOC_EXPERIMENT_H
#define INDUCTION_FOC_EXPERIMENT_H
#include "induction_experiment.h"
#include "induction_foc.h"
#include "induction_ifoc.h"

#define FOC_CURRENT_DT .0001
#define FOC_CURRENT_TICKS_PER_SPEED 10

typedef struct {
  InductionTrial plant;
  FocParams params;
  FocState controller;
  IfocEstimate estimate;
  int useIndirect;
  int useFoc, currentTicks, voltageLimitedTicks, currentLimitedSpeedTicks;
  double maxVoltage, maxReferenceCurrent, maxActualCurrent;
} FocExperiment;

static inline FocExperiment FocExperimentInit(int useFoc) {
  FocExperiment t = {0};
  t.plant = InductionTrialInit(0, 0); /* external speed ramp, no double ramp */
  t.useFoc = useFoc;
  t.params = FocDefaultParams(&t.plant.motor, t.plant.driveParams.maxPhasePeakV);
  FocInit(&t.controller);
  return t;
}

/* Nominal controller model is constructed separately from plant state.
 * A plant parameter change does not update this controller's assumptions.
 * rrScale is CONTROLLER Rr / nominal Rr; plant remains unchanged. */
static inline FocExperiment IfocExperimentInit(double rrScale) {
  FocExperiment t = FocExperimentInit(1);
  SimmyInductionParams nominal = InductionMotor();
  nominal.rotorResistance *= rrScale;
  t.params = FocDefaultParams(&nominal, 230*sqrt(2.0/3));
  t.useIndirect = 1;
  IfocInit(&t.estimate);
  return t;
}

/* Fixed 1 kHz speed/command tick, 10 kHz voltage/current ticks. Both drives
 * use the SAME held-voltage inverter interface. Refining stepsPerCurrent
 * changes only the plant solver, never controller sampling or gains. */
static inline void FocExperimentTick(FocExperiment *t, double speedReference,
                                     double brake, int stepsPerCurrent) {
  InductionTrial *p = &t->plant;
  p->chain.bodies[0].coulombLevel = p->chain.bodies[0].stictionLimit = brake;
  if (t->useFoc) {
    if (t->useIndirect) {
      IfocMeasurements measured = {.shaftSpeed=InductionSpeed(p)};
      IfocSpeedStep(&t->controller, &t->estimate, &t->params, &measured,
                    speedReference, INDUCTION_COMMAND_DT);
    } else {
      FocSpeedStep(&t->controller, &t->params, speedReference, InductionSpeed(p),
                  hypot(p->electrical.rotorFlux[0], p->electrical.rotorFlux[1]), INDUCTION_COMMAND_DT);
    }
    t->currentLimitedSpeedTicks += t->controller.currentLimited;
    t->maxReferenceCurrent = fmax(t->maxReferenceCurrent,
        hypot(t->controller.currentReference[0], t->controller.currentReference[1]));
  } else {
    SimmyVfdCommandStep(&p->drive, &p->driveParams,
                       speedReference*p->motor.polePairs/(2*INDUCTION_PI), INDUCTION_COMMAND_DT);
  }
  for (int tick = 0; tick < FOC_CURRENT_TICKS_PER_SPEED; tick++) {
    double voltage[2];
    if (t->useFoc) {
      SimmyInductionOutput output;
      SimmyInductionEvaluate(&p->electrical, &p->motor, &output);
      if (t->useIndirect) {
        IfocMeasurements measured = {.statorCurrent={output.statorCurrent[0], output.statorCurrent[1]},
                                     .shaftSpeed=InductionSpeed(p)};
        IfocCurrentStep(&t->controller, &t->estimate, &t->params, &measured, FOC_CURRENT_DT);
      } else {
        FocCurrentStep(&t->controller, &t->params, output.statorCurrent,
                      p->electrical.rotorFlux, InductionSpeed(p), FOC_CURRENT_DT);
      }
      voltage[0] = t->controller.voltage[0]; voltage[1] = t->controller.voltage[1];
      t->voltageLimitedTicks += t->controller.voltageLimited;
    } else {
      SimmyVfdVoltageStep(&p->drive, FOC_CURRENT_DT, voltage);
    }
    t->maxVoltage = fmax(t->maxVoltage, hypot(voltage[0], voltage[1]));
    for (int step = 0; step < stepsPerCurrent; step++) {
      SimmyInductionChainStep(&p->electrical, &p->motor, &p->mechanics, &p->chain,
          &p->mount, voltage[0], voltage[1], FOC_CURRENT_DT/stepsPerCurrent);
      SimmyInductionOutput output;
      SimmyInductionEvaluate(&p->electrical, &p->motor, &output);
      t->maxActualCurrent = fmax(t->maxActualCurrent,
                                hypot(output.statorCurrent[0], output.statorCurrent[1]));
    }
    t->currentTicks++;
  }
}

static inline double FocComparisonReference(double time) {
  return fmin(150, fmax(0, (time-.3)*60)); /* flux first; 60 rad/s^2 speed ramp */
}

static inline double FocStressReference(double time) {
  return time < .3 ? 0 : (time < 3.5 ? fmin(350, (time-.3)*200) : 100);
}

/* Estimation experiment isolates a load application/removal, rather than
 * repeating the oracle benchmark's overload/stiction recapture test. */
static inline double IfocComparisonBrake(double time) {
  return time >= 3.5 && time < 7.5 ? 6 : 0;
}

static inline double FocComparisonBrake(double time) {
  return time < 3.5 ? 0 : (time < 5.5 || time >= 7.5 ? 6 : 20);
}
#endif
