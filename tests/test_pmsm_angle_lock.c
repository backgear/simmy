#include "../examples/pmsm_angle_lock.h"
#include <assert.h>
#include <stdio.h>

static void near(double a, double b, double tolerance) { assert(fabs(a-b) < tolerance); }

#define VOLTAGE_MAGNITUDE 5.0
#define DRIVE_ANGLE 0.0

/* Runs until durationSeconds, tracking the first two POSITIVE->non-
 * positive crossings of (angle-equilibrium) to get a period estimate --
 * exactly the measure examples/plot_pmsm_angle_lock.py's own
 * first_period() uses, so the two agree. */
static void run(double inertiaKgM2, double startOffsetRad, double durationSeconds,
               double *firstCross, double *period, double *finalAngle, double *maxCurrent) {
  PmAngleLockTrial t = PmAngleLockTrialInit(DRIVE_ANGLE, VOLTAGE_MAGNITUDE, inertiaKgM2, 0);
  double equilibrium = PmAngleLockEquilibrium(DRIVE_ANGLE, &t.motor);
  PmAngleLockSetState(&t, equilibrium+startOffsetRad, 0);

  double prevError = startOffsetRad;
  double crossings[4]; int crossCount = 0;
  *maxCurrent = 0;
  int steps = (int)(durationSeconds/PMSM_ANGLE_LOCK_DT);
  for (int i = 0; i < steps; i++) {
    SimmyPmOutput output;
    assert(PmAngleLockTick(&t, &output));
    double error = PmAngleLockAngle(&t)-equilibrium;
    double current = hypot(t.electrical.current[0], t.electrical.current[1]);
    if (current > *maxCurrent) *maxCurrent = current;
    assert(isfinite(error) && isfinite(current));
    if (crossCount < 4 && prevError > 0 && error <= 0)
      crossings[crossCount++] = (i+1)*PMSM_ANGLE_LOCK_DT;
    prevError = error;
  }
  *finalAngle = PmAngleLockAngle(&t);
  *firstCross = crossCount > 0 ? crossings[0] : -1;
  *period = crossCount > 1 ? crossings[1]-crossings[0] : -1;
}

/* Same restoring-torque law, same fixed drive vector, four rotor
 * inertias spanning two orders of magnitude (125x): the settled position
 * must be identical and the oscillation period must grow like sqrt(J),
 * the textbook damped-second-order-system result derived in
 * examples/pmsm_angle_lock.h's file header. */
static void settling_and_period_scaling(void) {
  double inertias[] = {.0002, .001, .005, .025};
  double bem = PmAngleLockElectricalDampingNm(
      &(SimmyPmParams){.resistance=.6, .backEmfConstant=.08});
  double periods[4], finals[4];
  for (int i = 0; i < 4; i++) {
    double tau = 2*inertias[i]/bem;
    double duration = fmax(2.0, 10*tau); /* >99.99% envelope decay */
    double firstCross, period, finalAngle, maxCurrent;
    run(inertias[i], PMSM_ANGLE_LOCK_PI/8, duration, &firstCross, &period, &finalAngle, &maxCurrent);
    assert(firstCross > 0 && period > 0);
    /* Fully settled back to the SAME equilibrium regardless of inertia. */
    near(finalAngle, 0, 1e-3);
    /* Reasonable, finite current -- an open-loop pulse, not a claim
     * about any current limit (there is no controller here to enforce
     * one). */
    assert(maxCurrent > 0 && maxCurrent < 15);
    periods[i] = period;
    finals[i] = finalAngle;
    printf("J=%.4f kg*m^2: period %.4f s, final angle %.2e rad, peak current %.3f A\n",
           inertias[i], period, finalAngle, maxCurrent);
  }
  (void)finals;

  /* Periods must grow monotonically with inertia. */
  for (int i = 1; i < 4; i++) assert(periods[i] > periods[i-1]);

  /* sqrt(J) scaling, consecutive 5x-inertia steps -> sqrt(5)=2.236x
   * period. Tolerance is loose (15%) because the quasi-static
   * restoring-torque approximation (see the header derivation) only
   * holds once the mechanical period is large relative to this motor's
   * own L/R~3.3ms electrical time constant -- truest for the larger
   * inertias, measurably (but boundedly) less so for the smallest. */
  for (int i = 1; i < 4; i++) {
    double ratioJ = sqrt(inertias[i]/inertias[i-1]);
    double ratioPeriod = periods[i]/periods[i-1];
    double relativeError = fabs(ratioPeriod-ratioJ)/ratioJ;
    assert(relativeError < .15);
  }
}

/* The equilibrium does not depend on which side the rotor starts
 * displaced from -- a basic sanity check that this is a real restoring
 * torque, not an artifact of the starting sign. */
static void direction_symmetry(void) {
  double firstCross, period, finalAnglePositive, finalAngleNegative, maxCurrent;
  run(.001, PMSM_ANGLE_LOCK_PI/8, 2.0, &firstCross, &period, &finalAnglePositive, &maxCurrent);
  run(.001, -PMSM_ANGLE_LOCK_PI/8, 2.0, &firstCross, &period, &finalAngleNegative, &maxCurrent);
  near(finalAnglePositive, 0, 1e-3);
  near(finalAngleNegative, 0, 1e-3);
}

int main(void) {
  settling_and_period_scaling();
  direction_symmetry();
  puts("PMSM angle-lock tests passed.");
  return 0;
}
