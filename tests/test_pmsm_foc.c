#include "../examples/pmsm_foc.h"
#include <assert.h>
#include <stdio.h>

static void near(double a, double b, double tolerance) { assert(fabs(a-b) < tolerance); }

static void transforms_and_torque(void) {
  SimmyPmParams motor = {.polePairs=4, .waveform=SIMMY_PM_SINE, .resistance=.6,
      .inductance=.002, .backEmfConstant=.08};
  for (int i = -30; i <= 30; i++) {
    double angle = i*.27, dq[2] = {.3, i*.05}, ab[2], recovered[2];
    FocInversePark(dq, angle, ab); FocPark(ab, angle, recovered);
    near(dq[0], recovered[0], 1e-14); near(dq[1], recovered[1], 1e-14);
    near(hypot(dq[0], dq[1]), hypot(ab[0], ab[1]), 1e-14);
    /* The plant's own torque, driven with the inverse-Parked currents at
     * ZERO speed (so only the resistive/no-back-EMF terminal matters),
     * must equal 1.5*Ke*iq -- the same Nm/A convention FocDefaultParams
     * assumes for every other FOC example in this directory. */
    SimmyPmState state = {.current = {ab[0], ab[1]}};
    SimmyPmOutput output;
    assert(SimmyPmStep(&state, &motor, ab, angle/motor.polePairs, 0, 1e-9, &output));
    near(output.torque, 1.5*motor.backEmfConstant*dq[1], 1e-6);
  }
}

static void current_loop_and_limits(void) {
  SimmyPmParams motor = {.polePairs=4, .waveform=SIMMY_PM_SINE, .resistance=.6,
      .inductance=.002, .backEmfConstant=.08};
  PmFocParams p = PmFocDefaultParams(&motor, 24/sqrt(3.0));
  PmFocState s; PmFocInit(&s);

  /* Id reference is always 0 (MTPA for this non-salient plant); the speed
   * loop only ever sets Iq. */
  PmFocSpeedStep(&s, &p, 1000, 0, .001);
  assert(s.currentReference[0] == 0);
  assert(s.currentLimited);
  near(s.currentReference[1], p.currentLimit, 1e-13);

  /* Current-loop voltage projection is independent of the mechanical
   * solver: a big current step at zero speed saturates the voltage
   * vector exactly to the configured limit. */
  double currents[2] = {0, 0};
  PmFocCurrentStep(&s, &p, currents, 0, 0, .0001);
  assert(s.voltageLimited);
  near(hypot(s.voltage[0], s.voltage[1]), p.voltageLimit, 1e-12);
  assert(isfinite(s.currentIntegral[0]) && isfinite(s.currentIntegral[1]));

  /* Outer integrator freezes when voltage cannot support further effort. */
  PmFocInit(&s); s.voltageLimited = 1;
  PmFocSpeedStep(&s, &p, 1, 0, .001);
  assert(!s.currentLimited && s.speedIntegral == 0);
  s.voltageLimited = 0;
  PmFocSpeedStep(&s, &p, 1, 0, .001);
  assert(s.speedIntegral > 0);

  /* Back-calculation is actually active, not just a voltage output clamp. */
  PmFocInit(&s);
  currents[0] = 0; currents[1] = 0;
  s.currentIntegral[0] = 1000;
  PmFocCurrentStep(&s, &p, currents, 0, 0, .0001);
  near(s.currentIntegral[0], 1000+.0001*p.currentAw*(p.voltageLimit-1000), 1e-9);
}

static void assert_limits(const PmFocTrial *t) {
  assert(t->maxReferenceCurrent <= t->params.currentLimit + 1e-12);
  assert(t->maxVoltage <= t->params.voltageLimit + 1e-10);
  assert(isfinite(t->controller.speedIntegral));
  assert(isfinite(t->controller.currentIntegral[0]) && isfinite(t->controller.currentIntegral[1]));
  /* Reference limit is NOT a hard clamp on physical current. */
  assert(isfinite(t->maxActualCurrent));
}

static void speed_tracking_and_load_rejection(void) {
  PmFocTrial t = PmFocTrialInit();
  for (int tick = 0; tick < 5000; tick++) {
    double time = tick*.001;
    PmFocTrialTick(&t, PmFocComparisonReference(time), PmFocComparisonBrake(time));
    if (tick == 1499) near(PmFocTrialSpeed(&t), 120, .01); /* settled, no load yet */
    if (tick == 3499) near(PmFocTrialSpeed(&t), 120, .01); /* settled under the 0.3 Nm brake */
  }
  near(PmFocTrialSpeed(&t), 120, .01); /* brake released again by the end */
  /* Within this sample motor's voltage budget the whole run; a software
   * reference limit is not a hardware current trip, but this profile
   * never needs to find out. */
  assert(t.voltageLimitedTicks == 0);
  assert_limits(&t);
}

static void voltage_saturation_and_recovery(void) {
  PmFocTrial t = PmFocTrialInit();
  for (int tick = 0; tick < 4000; tick++) {
    PmFocTrialTick(&t, PmFocStressReference(tick*.001), 0);
    if (tick == 1999) {
      /* 300 rad/s is above the ~170 rad/s ceiling this bus/Ke combination
       * can sustain; the controller saturates rather than tracking it. */
      assert(t.voltageLimitedTicks > 1000);
      assert(t.controller.voltageLimited);
      assert(PmFocTrialSpeed(&t) < 165);
      assert(fabs(t.controller.currentIntegral[1]) < 400);
    }
  }
  /* Switched to a reachable 100 rad/s at t=2s; anti-windup lets it recover
   * cleanly rather than staying wound up against the old unreachable one. */
  near(PmFocTrialSpeed(&t), 100, .01);
  assert(!t.controller.voltageLimited && !t.controller.currentLimited);
  assert(fabs(t.controller.speedIntegral) < 1);
  assert_limits(&t);
  /* Deliberately retain and expose the current overshoot on the abrupt
   * post-saturation reference change: a software reference limit is not
   * hardware overcurrent protection. */
  assert(t.maxActualCurrent > t.params.currentLimit);
  assert(t.maxActualCurrent < 2*t.params.currentLimit);
  printf("PMSM FOC voltage ceiling: %d saturated current ticks, then recovered to %.4f rad/s\n",
         t.voltageLimitedTicks, PmFocTrialSpeed(&t));
}

static void direction_and_sensed_angle(void) {
  PmFocTrial forward = PmFocTrialInit(), reverse = PmFocTrialInit();
  double maxAngleErrorDeg = 0;
  for (int tick = 0; tick < 3000; tick++) {
    double time = tick*.001, ref = PmFocComparisonReference(time), brake = PmFocComparisonBrake(time);
    PmFocTrialTick(&forward, ref, brake);
    PmFocTrialTick(&reverse, -ref, brake);
    double angle = PmFocTrialAngle(&forward);
    double ideal = remainder(forward.motor.polePairs*angle+forward.motor.electricalOffset, 2*FOC_PI);
    double sensed = PmResolverSensedAngle(angle, forward.motor.polePairs,
                                          forward.motor.electricalOffset, &forward.resolver);
    maxAngleErrorDeg = fmax(maxAngleErrorDeg, fabs(remainder(sensed-ideal, 2*FOC_PI)*180/FOC_PI));
  }
  near(PmFocTrialSpeed(&forward), 120, .01);
  near(PmFocTrialSpeed(&reverse), -120, .01);
  near(PmFocTrialSpeed(&forward), -PmFocTrialSpeed(&reverse), 1e-4);
  /* The resolver stand-in's static gain/offset errors (see
   * PmResolverSensedAngle) stay bounded -- it has no unbounded drift. */
  assert(maxAngleErrorDeg < 1.5);
}

int main(void) {
  transforms_and_torque(); current_loop_and_limits();
  speed_tracking_and_load_rejection(); voltage_saturation_and_recovery();
  direction_and_sensed_angle();
  puts("PMSM FOC tests passed.");
  return 0;
}
