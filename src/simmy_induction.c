#include "simmy.h"
#include <math.h>

#define TWO_PI 6.28318530717958647692

int SimmyInductionParamsValid(const SimmyInductionParams *p) {
  return p->polePairs > 0 && isfinite(p->statorResistance) && p->statorResistance >= 0 &&
      isfinite(p->rotorResistance) && p->rotorResistance >= 0 &&
      isfinite(p->statorLeakageInductance) && p->statorLeakageInductance > 0 &&
      isfinite(p->rotorLeakageInductance) && p->rotorLeakageInductance > 0 &&
      isfinite(p->magnetizingInductance) && p->magnetizingInductance > 0;
}

void SimmyInductionInit(SimmyInductionState *s) { *s = (SimmyInductionState){0}; }

void SimmyInductionEvaluate(const SimmyInductionState *s,
                            const SimmyInductionParams *p, SimmyInductionOutput *o) {
  double lm = p->magnetizingInductance;
  double ls = lm + p->statorLeakageInductance;
  double lr = lm + p->rotorLeakageInductance;
  /* Expanded form avoids subtracting nearly equal Ls*Lr and Lm^2. */
  double determinant = p->statorLeakageInductance*p->rotorLeakageInductance +
      lm*(p->statorLeakageInductance+p->rotorLeakageInductance);
  o->magneticEnergy = 0;
  for (int axis = 0; axis < 2; axis++) {
    o->statorCurrent[axis] = (lr*s->statorFlux[axis]-lm*s->rotorFlux[axis])/determinant;
    o->rotorCurrent[axis] = (ls*s->rotorFlux[axis]-lm*s->statorFlux[axis])/determinant;
    o->magneticEnergy += .75*(s->statorFlux[axis]*o->statorCurrent[axis] +
                            s->rotorFlux[axis]*o->rotorCurrent[axis]);
  }
  o->torque = 1.5*p->polePairs*(s->statorFlux[0]*o->statorCurrent[1] -
                              s->statorFlux[1]*o->statorCurrent[0]);
}

static SimmyInductionState Derivative(const SimmyInductionState *s,
                                      const SimmyInductionParams *p,
                                      double va, double vb, double speed, double *torque) {
  SimmyInductionOutput o;
  SimmyInductionEvaluate(s, p, &o);
  *torque = o.torque;
  SimmyInductionState d;
  d.statorFlux[0] = va - p->statorResistance*o.statorCurrent[0];
  d.statorFlux[1] = vb - p->statorResistance*o.statorCurrent[1];
  double we = p->polePairs*speed;
  d.rotorFlux[0] = -p->rotorResistance*o.rotorCurrent[0] - we*s->rotorFlux[1];
  d.rotorFlux[1] = -p->rotorResistance*o.rotorCurrent[1] + we*s->rotorFlux[0];
  return d;
}

static SimmyInductionState Offset(const SimmyInductionState *s,
                                  const SimmyInductionState *d, double dt) {
  SimmyInductionState result;
  for (int axis = 0; axis < 2; axis++) {
    result.statorFlux[axis] = s->statorFlux[axis] + dt*d->statorFlux[axis];
    result.rotorFlux[axis] = s->rotorFlux[axis] + dt*d->rotorFlux[axis];
  }
  return result;
}

double SimmyInductionStep(SimmyInductionState *s, const SimmyInductionParams *p,
                          double va, double vb, double speed, double dt) {
  double t1, t2, t3, t4;
  SimmyInductionState k1 = Derivative(s, p, va, vb, speed, &t1);
  SimmyInductionState temp = Offset(s, &k1, dt*.5);
  SimmyInductionState k2 = Derivative(&temp, p, va, vb, speed, &t2);
  temp = Offset(s, &k2, dt*.5);
  SimmyInductionState k3 = Derivative(&temp, p, va, vb, speed, &t3);
  temp = Offset(s, &k3, dt);
  SimmyInductionState k4 = Derivative(&temp, p, va, vb, speed, &t4);
  for (int axis = 0; axis < 2; axis++) {
    s->statorFlux[axis] += dt/6*(k1.statorFlux[axis]+2*k2.statorFlux[axis]+2*k3.statorFlux[axis]+k4.statorFlux[axis]);
    s->rotorFlux[axis] += dt/6*(k1.rotorFlux[axis]+2*k2.rotorFlux[axis]+2*k3.rotorFlux[axis]+k4.rotorFlux[axis]);
  }
  return (t1+2*t2+2*t3+t4)/6;
}

void SimmyInductionChainStep(SimmyInductionState *state, const SimmyInductionParams *motor,
                             SimmyChainState *chainState, const SimmyChainParams *chain,
                             const SimmyMotorMount *mount, double va, double vb, double dt) {
  double speed = chainState->bodies[0].velocity*mount->radiansPerCount/mount->tickSeconds;
  double torque = SimmyInductionStep(state, motor, va, vb, speed, dt);
  SimmyChainStepDt(chainState, chain, torque*mount->forcePerNm, dt/mount->tickSeconds);
}

void SimmyVfdInit(SimmyVfdState *s) { *s = (SimmyVfdState){0}; }

void SimmyVfdCommandStep(SimmyVfdState *s, const SimmyVfdParams *p,
                         double target, double dt) {
  double difference = target-s->frequencyHz;
  if (p->rampHzPerSecond > 0) {
    double limit = p->rampHzPerSecond*dt;
    difference = fmax(-limit, fmin(limit, difference));
  }
  s->frequencyHz += difference;
  double voltage = p->boostPhasePeakV +
      (p->basePhasePeakV-p->boostPhasePeakV)*fabs(s->frequencyHz)/p->baseFrequencyHz;
  s->phasePeakV = s->frequencyHz == 0 ? 0 : fmin(p->maxPhasePeakV, voltage);
}

void SimmyVfdVoltageStep(SimmyVfdState *s, double dt, double voltage[2]) {
  double advance = TWO_PI*s->frequencyHz*dt;
  double midpoint = s->phaseRadians + .5*advance;
  voltage[0] = s->phasePeakV*cos(midpoint);
  voltage[1] = s->phasePeakV*sin(midpoint);
  s->phaseRadians = remainder(s->phaseRadians+advance, TWO_PI);
}
