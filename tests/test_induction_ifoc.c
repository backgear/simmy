#include "../examples/induction_foc_experiment.h"
#include <assert.h>
#include <stdio.h>

static void near(double a, double b, double tolerance) { assert(fabs(a-b) < tolerance); }

static double angle_error(const FocExperiment *t) {
  double truth = atan2(t->plant.electrical.rotorFlux[1], t->plant.electrical.rotorFlux[0]);
  return remainder(t->estimate.fluxAngle-truth, 2*FOC_PI);
}

static void scalar_model_and_slip(void) {
  FocExperiment fixture = IfocExperimentInit(1);
  FocParams p = fixture.params;
  IfocEstimate e; IfocInit(&e);
  IfocMeasurements m = {.statorCurrent={2,0}, .shaftSpeed=0};
  for (int i = 0; i < 1000; i++) IfocEstimateStep(&e, &p, &m, .0001);
  near(e.fluxMagnitude, p.lm*2*(1-exp(-.1/p.rotorTimeConstant)), 1e-13);
  near(e.fluxAngle, 0, 1e-14);
  e = (IfocEstimate){.fluxMagnitude=.5, .fluxAngle=.4};
  double dq[2] = {.5/p.lm, 3};
  FocInversePark(dq, e.fluxAngle, m.statorCurrent); m.shaftSpeed=20;
  IfocEstimateStep(&e, &p, &m, .0001);
  double slip = p.lm/p.rotorTimeConstant*3/.5;
  near(e.slipRadSec, slip, 1e-12);
  near(e.fluxAngle, .4+.0001*(p.polePairs*20+slip), 1e-13);
  near(e.fluxMagnitude, .5, 1e-14);

  /* Zero measured magnetizing current does not create flux just because
   * the controller requested it. Low-flux slip division stays disabled. */
  FocState control; FocInit(&control); IfocInit(&e);
  m = (IfocMeasurements){0};
  for (int i = 0; i < 1000; i++) {
    IfocSpeedStep(&control, &e, &p, &m, 150, .001);
    IfocCurrentStep(&control, &e, &p, &m, .0001);
  }
  assert(e.fluxMagnitude == 0 && e.slipRadSec == 0);
  assert(!control.fluxReady && control.currentReference[1] == 0);
  assert(control.currentReference[0] > 0);
}

static void truth_is_not_a_controller_input(void) {
  FocExperiment t = IfocExperimentInit(1);
  /* True pre-existing flux with zero estimated flux: indirect gating must
   * NOT release torque based on the hidden plant truth. Same current=0. */
  t.plant.electrical.rotorFlux[0] = .5;
  t.plant.electrical.statorFlux[0] = t.params.lm/t.params.lr*.5;
  FocExperimentTick(&t, 150, 0, 4);
  assert(hypot(t.plant.electrical.rotorFlux[0], t.plant.electrical.rotorFlux[1]) > .45);
  assert(!t.controller.fluxReady && t.controller.currentReference[1] == 0);
  assert(t.estimate.fluxMagnitude < .01);
  double tau = t.params.rotorTimeConstant;
  t.plant.motor.rotorResistance *= 2;
  assert(t.params.rotorTimeConstant == tau); /* no live plant-parameter copying */
}

static void tracking_and_mismatch(void) {
  FocExperiment ideal = FocExperimentInit(1), matched = IfocExperimentInit(1);
  FocExperiment low = IfocExperimentInit(.5), high = IfocExperimentInit(1.5);
  for (int tick = 0; tick < 11000; tick++) {
    double time = tick*.001, ref = FocComparisonReference(time), load = IfocComparisonBrake(time);
    FocExperimentTick(&ideal, ref, load, 4);
    FocExperimentTick(&matched, ref, load, 4);
    FocExperimentTick(&low, ref, load, 4);
    FocExperimentTick(&high, ref, load, 4);
    if (tick == 5499) {
      near(InductionSpeed(&matched.plant), 150, .01);
      near(angle_error(&matched), 0, .1*FOC_PI/180);
      near(matched.estimate.fluxMagnitude,
           hypot(matched.plant.electrical.rotorFlux[0], matched.plant.electrical.rotorFlux[1]), .001);
      assert(InductionSpeed(&low.plant) < 120);
      assert(hypot(low.plant.electrical.rotorFlux[0], low.plant.electrical.rotorFlux[1]) > .8);
      /* Good speed tracking can hide incorrect flux / torque estimates. */
      near(InductionSpeed(&high.plant), 150, .01);
      assert(fabs(angle_error(&high)) > 5*FOC_PI/180);
      double highTruth = hypot(high.plant.electrical.rotorFlux[0], high.plant.electrical.rotorFlux[1]);
      assert(high.estimate.fluxMagnitude-highTruth > .1);
    }
  }
  near(InductionSpeed(&matched.plant), InductionSpeed(&ideal.plant), .01);
  near(InductionSpeed(&low.plant), 150, .05);
  near(InductionSpeed(&high.plant), 150, .05);
  assert(matched.maxReferenceCurrent <= 8+1e-12);
  assert(low.maxReferenceCurrent <= 8+1e-12 && high.maxReferenceCurrent <= 8+1e-12);
  assert(low.maxVoltage <= low.params.voltageLimit+1e-10);
  printf("Indirect FOC: matched model tracks oracle; low/high assumed Rr expose speed/flux errors under load\n");
}

static void clocks_and_direction(void) {
  FocExperiment a = IfocExperimentInit(1), b = a;
  double maxDifference = 0;
  for (int tick = 0; tick < 4500; tick++) {
    double time = tick*.001, ref = FocComparisonReference(time), load = IfocComparisonBrake(time);
    FocExperimentTick(&a, ref, load, 4); FocExperimentTick(&b, ref, load, 8);
    maxDifference = fmax(maxDifference, fabs(InductionSpeed(&a.plant)-InductionSpeed(&b.plant)));
  }
  assert(a.currentTicks == b.currentTicks && a.currentTicks == 45000);
  assert(maxDifference < .002);
  near(angle_error(&a), angle_error(&b), .0001);

  FocExperiment forward = IfocExperimentInit(1), reverse = forward;
  for (int tick = 0; tick < 3000; tick++) {
    double ref = fmin(80, fmax(0, (tick*.001-.3)*60));
    FocExperimentTick(&forward, ref, 2, 4);
    FocExperimentTick(&reverse, -ref, 2, 4);
  }
  near(InductionSpeed(&forward.plant), 80, .01);
  near(InductionSpeed(&reverse.plant), -80, .01);
  near(InductionSpeed(&forward.plant), -InductionSpeed(&reverse.plant), 1e-8);
  near(forward.estimate.fluxMagnitude, reverse.estimate.fluxMagnitude, 1e-10);
}

int main(void) {
  scalar_model_and_slip(); truth_is_not_a_controller_input();
  tracking_and_mismatch(); clocks_and_direction();
  puts("Indirect induction FOC tests passed.");
  return 0;
}
