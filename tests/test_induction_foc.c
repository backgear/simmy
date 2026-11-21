#include "../examples/induction_foc_experiment.h"
#include <assert.h>
#include <stdio.h>

static void near(double a, double b, double tolerance) { assert(fabs(a-b) < tolerance); }

static void transforms_and_torque(void) {
  SimmyInductionParams motor = InductionMotor();
  FocParams p = FocDefaultParams(&motor, 187);
  for (int i = -30; i <= 30; i++) {
    double angle = i*.27, dq[2] = {2, i*.1}, ab[2], recovered[2];
    FocInversePark(dq, angle, ab); FocPark(ab, angle, recovered);
    near(dq[0], recovered[0], 1e-14); near(dq[1], recovered[1], 1e-14);
    near(hypot(dq[0], dq[1]), hypot(ab[0], ab[1]), 1e-14);
    SimmyInductionState state;
    state.rotorFlux[0] = .4*cos(angle); state.rotorFlux[1] = .4*sin(angle);
    for (int axis = 0; axis < 2; axis++)
      state.statorFlux[axis] = p.leakage*ab[axis] + p.lm/p.lr*state.rotorFlux[axis];
    SimmyInductionOutput output;
    SimmyInductionEvaluate(&state, &motor, &output);
    near(output.torque, 1.5*p.polePairs*p.lm/p.lr*.4*dq[1], 1e-13);
  }
}

static void gate_and_limits(void) {
  SimmyInductionParams motor = InductionMotor();
  FocParams p = FocDefaultParams(&motor, 187);
  FocState s; FocInit(&s);
  FocSpeedStep(&s, &p, 1000, 0, 0, .001);
  assert(!s.fluxReady && s.currentReference[1] == 0 && s.speedIntegral == 0);
  near(s.currentReference[0], p.fluxReference/p.lm, 1e-14);
  FocSpeedStep(&s, &p, 1000, 0, .449, .001);
  assert(!s.fluxReady && s.torqueRequest == 0);
  FocSpeedStep(&s, &p, 1000, 0, .5, .001);
  assert(s.fluxReady && s.currentLimited);
  near(hypot(s.currentReference[0], s.currentReference[1]), p.currentLimit, 1e-13);
  /* Flux loss inhibits torque again, not a division by zero / stale angle. */
  FocSpeedStep(&s, &p, 1000, 0, .1, .001);
  assert(!s.fluxReady && s.currentReference[1] == 0);

  /* Current-loop voltage projection independent of the mechanical solver. */
  double currents[2] = {0, 0}, flux[2] = {.5, 0};
  FocSpeedStep(&s, &p, 1000, 0, .5, .001);
  FocCurrentStep(&s, &p, currents, flux, 1000, .0001);
  assert(s.voltageLimited);
  near(hypot(s.voltage[0], s.voltage[1]), p.voltageLimit, 1e-12);
  assert(isfinite(s.currentIntegral[0]) && isfinite(s.currentIntegral[1]));

  /* Outer integrator freezes when voltage cannot support further effort,
   * even if the current-reference circle itself is not saturated. */
  FocInit(&s); s.voltageLimited = 1;
  FocSpeedStep(&s, &p, 1, 0, .5, .001);
  assert(!s.currentLimited && s.speedIntegral == 0);
  s.voltageLimited = 0;
  FocSpeedStep(&s, &p, 1, 0, .5, .001);
  assert(s.speedIntegral > 0);

  /* Back-calculation is actually active, not just a voltage output clamp. */
  FocInit(&s);
  s.currentReference[0] = p.fluxReference/p.lm;
  currents[0] = s.currentReference[0]; currents[1] = 0;
  s.currentIntegral[0] = 1000;
  FocCurrentStep(&s, &p, currents, flux, 0, .0001);
  near(s.currentIntegral[0], 1000+.0001*p.currentAw*(p.voltageLimit-1000), 1e-9);
}

static void assert_limits(const FocExperiment *t) {
  assert(t->maxReferenceCurrent <= t->params.currentLimit + 1e-12);
  assert(t->maxVoltage <= t->params.voltageLimit + 1e-10);
  assert(isfinite(t->controller.speedIntegral));
  assert(isfinite(t->controller.currentIntegral[0]) && isfinite(t->controller.currentIntegral[1]));
  /* Reference limit is NOT a hard clamp on physical current. Voltage
   * clipping can prevent tracking; never clip the plant's state. */
  assert(isfinite(t->maxActualCurrent));
}

static void buildup_with_pending_demand(void) {
  FocExperiment t = FocExperimentInit(1);
  int released = 0;
  for (int tick = 0; tick < 2000; tick++) {
    FocExperimentTick(&t, 100, 0, 4); /* nonzero speed requested from t=0 */
    if (!t.controller.fluxReady) {
      assert(t.controller.currentReference[1] == 0);
      near(InductionSpeed(&t.plant), 0, 1e-12);
    } else if (!released) {
      assert(tick > 200 && tick < 300); released = 1;
    }
  }
  assert(released);
  near(InductionSpeed(&t.plant), 100, .01);
}

static void buildup_and_load_rejection(void) {
  FocExperiment foc = FocExperimentInit(1), vhz = FocExperimentInit(0);
  int released = 0;
  double minimumAfterLoad = 150, stalledFlux = 0;
  for (int tick = 0; tick < 11000; tick++) {
    double time = tick*.001;
    double ref = FocComparisonReference(time), load = FocComparisonBrake(time);
    FocExperimentTick(&foc, ref, load, 4);
    FocExperimentTick(&vhz, ref, load, 4);
    if (foc.controller.fluxReady && !released) {
      assert(time > .2 && time < .3); released = 1;
    }
    if (tick < 200) {
      assert(!foc.controller.fluxReady);
      assert(foc.controller.currentReference[1] == 0);
      near(InductionSpeed(&foc.plant), 0, 1e-12);
    }
    if (tick >= 3500 && tick < 5500)
      minimumAfterLoad = fmin(minimumAfterLoad, InductionSpeed(&foc.plant));
    if (tick == 5499) {
      near(InductionSpeed(&foc.plant), 150, .005);
      assert(InductionSpeed(&vhz.plant) < 145);
    }
    if (tick == 7499) {
      assert(InductionSpeed(&foc.plant) == 0 && InductionSpeed(&vhz.plant) == 0);
      stalledFlux = hypot(foc.plant.electrical.rotorFlux[0], foc.plant.electrical.rotorFlux[1]);
      near(stalledFlux, .5, .002);
      assert(foc.controller.currentLimited);
    }
  }
  assert(released);
  assert(minimumAfterLoad > 143); /* finite droop, no instantaneous speed assignment */
  near(InductionSpeed(&foc.plant), 150, .005);
  assert(InductionSpeed(&vhz.plant) < 145);
  assert(foc.currentLimitedSpeedTicks > 1000);
  assert(foc.currentTicks == 11000*FOC_CURRENT_TICKS_PER_SPEED);
  assert_limits(&foc);
  assert(foc.maxActualCurrent < 8.1); /* ordinary load profile, not a hard current guarantee */
  printf("FOC: 6 Nm load rejected, minimum speed %.3f rad/s; overload stalls with flux %.4f; recovery %.4f rad/s\n",
         minimumAfterLoad, stalledFlux, InductionSpeed(&foc.plant));
}

static void voltage_saturation_and_recovery(void) {
  FocExperiment t = FocExperimentInit(1);
  for (int tick = 0; tick < 6000; tick++) {
    FocExperimentTick(&t, FocStressReference(tick*.001), 0, 4);
    if (tick == 3499) {
      assert(t.voltageLimitedTicks > 10000);
      assert(t.controller.voltageLimited);
      assert(InductionSpeed(&t.plant) < 220); /* unattainable 350 rad/s */
      assert(fabs(t.controller.currentIntegral[1]) < 400);
    }
  }
  near(InductionSpeed(&t.plant), 100, .005);
  assert(!t.controller.voltageLimited && !t.controller.currentLimited);
  assert(fabs(t.controller.speedIntegral) < 1);
  assert(fabs(t.controller.currentIntegral[0]) < 20);
  assert(fabs(t.controller.currentIntegral[1]) < 20);
  assert_limits(&t);
  /* Deliberately retain and expose the current overshoot on the abrupt
   * post-saturation reference change: software reference limits are not
   * hardware overcurrent protection. */
  assert(t.maxActualCurrent > t.params.currentLimit);
  assert(t.maxActualCurrent < 2*t.params.currentLimit);
  printf("FOC voltage ceiling: %d saturated current ticks, then recovered to %.4f rad/s\n",
         t.voltageLimitedTicks, InductionSpeed(&t.plant));
}

static void resolution_and_direction(void) {
  FocExperiment coarse = FocExperimentInit(1), fine = coarse;
  double largestSpeedDifference = 0;
  for (int tick = 0; tick < 4500; tick++) {
    double time = tick*.001;
    FocExperimentTick(&coarse, FocComparisonReference(time), FocComparisonBrake(time), 4);
    FocExperimentTick(&fine, FocComparisonReference(time), FocComparisonBrake(time), 8);
    largestSpeedDifference = fmax(largestSpeedDifference,
        fabs(InductionSpeed(&coarse.plant)-InductionSpeed(&fine.plant)));
  }
  assert(coarse.currentTicks == fine.currentTicks);
  assert(largestSpeedDifference < .02); /* fixed controller clocks, refined plant only */

  FocExperiment forward = FocExperimentInit(1), reverse = forward;
  for (int tick = 0; tick < 3000; tick++) {
    double reference = fmin(80, fmax(0, (tick*.001-.3)*60));
    FocExperimentTick(&forward, reference, 2, 4);
    FocExperimentTick(&reverse, -reference, 2, 4);
  }
  near(InductionSpeed(&forward.plant), 80, .01);
  near(InductionSpeed(&reverse.plant), -80, .01);
  near(InductionSpeed(&forward.plant), -InductionSpeed(&reverse.plant), 1e-8);
}

int main(void) {
  transforms_and_torque(); gate_and_limits(); buildup_with_pending_demand(); buildup_and_load_rejection();
  voltage_saturation_and_recovery(); resolution_and_direction();
  puts("Ideal induction FOC tests passed.");
  return 0;
}
