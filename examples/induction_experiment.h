#ifndef INDUCTION_EXPERIMENT_H
#define INDUCTION_EXPERIMENT_H

/* Illustrative parameters, not an identified commercial motor. All physical
 * rates are SI; chain units refer permanently to a 1 ms tick, even when
 * refining integration. Command updates always remain 1 kHz. */
#include "simmy.h"
#include <math.h>

#define INDUCTION_PI 3.14159265358979323846
#define INDUCTION_COMMAND_DT .001

static inline SimmyInductionParams InductionMotor(void) {
  SimmyInductionParams p = {.polePairs=2, .statorResistance=3, .rotorResistance=2,
      .statorLeakageInductance=.01, .rotorLeakageInductance=.01, .magnetizingInductance=.18};
  return p;
}

typedef struct {
  SimmyInductionParams motor;
  SimmyInductionState electrical;
  SimmyVfdState drive;
  SimmyVfdParams driveParams;
  SimmyChainState mechanics;
  SimmyChainParams chain;
  SimmyMotorMount mount;
} InductionTrial;

static inline InductionTrial InductionTrialInit(double boost, double ramp) {
  InductionTrial t = {0};
  t.motor = InductionMotor();
  t.driveParams = (SimmyVfdParams){.baseFrequencyHz=50, .basePhasePeakV=230*sqrt(2.0/3),
      .maxPhasePeakV=230*sqrt(2.0/3), .boostPhasePeakV=boost, .rampHzPerSecond=ramp};
  t.mount = (SimmyMotorMount){.radiansPerCount=.001, .forcePerNm=1, .tickSeconds=.001};
  t.chain.bodyCount = 1;
  t.chain.subSteps = 1;
  t.chain.bodies[0].inertia = .02/(1000*.001*.001); /* J=.02 kg m^2 */
  t.chain.bodies[0].friction = .001/(1000*.001); /* B=.001 Nm/(rad/s) */
  SimmyInductionInit(&t.electrical);
  SimmyVfdInit(&t.drive);
  SimmyChainInit(&t.mechanics, &t.chain);
  return t;
}

static inline double InductionSpeed(const InductionTrial *t) {
  return t->mechanics.bodies[0].velocity*t->mount.radiansPerCount/t->mount.tickSeconds;
}

/* One fixed 1 ms command interval. The number of plant integration steps
 * is independent of that schedule. No rotor feedback reaches the drive. */
static inline void InductionTrialTick(InductionTrial *t, double frequencyHz, int integrationSteps) {
  SimmyVfdCommandStep(&t->drive, &t->driveParams, frequencyHz, INDUCTION_COMMAND_DT);
  double dt = INDUCTION_COMMAND_DT/integrationSteps;
  for (int i = 0; i < integrationSteps; i++) {
    double voltage[2];
    SimmyVfdVoltageStep(&t->drive, dt, voltage);
    SimmyInductionChainStep(&t->electrical, &t->motor, &t->mechanics, &t->chain,
                            &t->mount, voltage[0], voltage[1], dt);
  }
}
#endif
