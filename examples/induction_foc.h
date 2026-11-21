#ifndef INDUCTION_FOC_H
#define INDUCTION_FOC_H

/* Example controller, NOT a new plant or production/sensorless FOC claim.
 * SI, amplitude-invariant Clarke/Park. Shared current control accepts a
 * supplied flux frame: true in the oracle, estimated in indirect FOC.
 * Measured stator current remains plant output; no physical state clamp. */
#include "simmy.h"
#include <math.h>

#define FOC_PI 3.14159265358979323846

typedef struct {
  double polePairs, lm, lr, leakage, rotorTimeConstant;
  double fluxReference, currentLimit, voltageLimit;
  double currentKp, currentKi, currentAw;
  double speedKp, speedKi, speedAw;
} FocParams;

typedef struct {
  double currentIntegral[2], speedIntegral;
  double currentReference[2], measuredCurrent[2];
  double torqueRequest, voltage[2], fluxMagnitude;
  int fluxReady, voltageLimited, currentLimited;
} FocState;

static inline void FocPark(const double ab[2], double angle, double dq[2]) {
  double c = cos(angle), s = sin(angle);
  dq[0] = c*ab[0]+s*ab[1];
  dq[1] = -s*ab[0]+c*ab[1];
}

static inline void FocInversePark(const double dq[2], double angle, double ab[2]) {
  double c = cos(angle), s = sin(angle);
  ab[0] = c*dq[0]-s*dq[1];
  ab[1] = s*dq[0]+c*dq[1];
}

static inline double FocClamp(double value, double limit) {
  return fmax(-limit, fmin(limit, value));
}

static inline FocParams FocDefaultParams(const SimmyInductionParams *motor, double voltageLimit) {
  double lr = motor->magnetizingInductance + motor->rotorLeakageInductance;
  double leakage = motor->statorLeakageInductance +
      motor->magnetizingInductance*motor->rotorLeakageInductance/lr;
  double bandwidth = 2*FOC_PI*300;
  double speedBandwidth = 2*FOC_PI*5, inertia = .02; /* matches example mechanics */
  FocParams p = {.polePairs=motor->polePairs, .lm=motor->magnetizingInductance,
      .lr=lr, .leakage=leakage, .rotorTimeConstant=lr/motor->rotorResistance,
      .fluxReference=.5, .currentLimit=8, .voltageLimit=voltageLimit,
      .currentKp=bandwidth*leakage, .currentKi=bandwidth*motor->statorResistance,
      .currentAw=bandwidth, .speedKp=2*.9*speedBandwidth*inertia,
      .speedKi=speedBandwidth*speedBandwidth*inertia, .speedAw=50};
  return p;
}

static inline void FocInit(FocState *s) { *s = (FocState){0}; }

/* 1 kHz in the experiment. Vector current-reference limit gives d-axis
 * flux priority. Outer anti-windup tracks current-limited torque and
 * conditionally inhibits speed-error integration under voltage saturation.
 * Valid params: positive Rr, flux/current/voltage/gains; fluxRef/Lm < Imax. */
static inline void FocSpeedStep(FocState *s, const FocParams *p, double reference,
                                double speed, double fluxMagnitude, double dt) {
  s->fluxMagnitude = fluxMagnitude;
  if (fluxMagnitude >= .9*p->fluxReference) s->fluxReady = 1;
  if (fluxMagnitude < .5*p->fluxReference) s->fluxReady = 0;
  s->currentReference[0] = p->fluxReference/p->lm;
  if (!s->fluxReady) {
    s->speedIntegral = s->currentReference[1] = s->torqueRequest = 0;
    s->currentLimited = 0;
    return;
  }
  double torquePerAmp = 1.5*p->polePairs*p->lm/p->lr*fluxMagnitude;
  double iqLimit = sqrt(fmax(0, p->currentLimit*p->currentLimit -
                                s->currentReference[0]*s->currentReference[0]));
  double error = reference-speed;
  double raw = p->speedKp*error+s->speedIntegral;
  s->torqueRequest = FocClamp(raw, torquePerAmp*iqLimit);
  s->currentReference[1] = s->torqueRequest/torquePerAmp;
  s->currentLimited = fabs(raw) > torquePerAmp*iqLimit;
  double integration = p->speedKi*error;
  if (s->voltageLimited && raw*error > 0) integration = 0;
  s->speedIntegral += dt*(integration+p->speedAw*(s->torqueRequest-raw));
}

/* 10 kHz current control in a supplied flux frame. This core does not
 * know whether that frame is true or estimated. PI + decoupling, voltage
 * projection and back-calculation; output is held alpha/beta voltage. */
static inline void FocCurrentStepFrame(FocState *s, const FocParams *p,
                                       const double current[2], double flux, double angle,
                                       double speed, double dt) {
  FocPark(current, angle, s->measuredCurrent);
  double id = s->measuredCurrent[0], iq = s->measuredCurrent[1];
  double we = p->polePairs*speed;
  if (flux > .1*p->fluxReference) we += p->lm/p->rotorTimeConstant*iq/flux;
  double feedforward[2] = {
      p->lm/p->lr*(p->lm*id-flux)/p->rotorTimeConstant-we*p->leakage*iq,
      we*(p->leakage*id+p->lm/p->lr*flux)};
  double raw[2], error[2], applied[2];
  for (int axis = 0; axis < 2; axis++) {
    error[axis] = s->currentReference[axis]-s->measuredCurrent[axis];
    raw[axis] = p->currentKp*error[axis]+s->currentIntegral[axis]+feedforward[axis];
  }
  double magnitude = hypot(raw[0], raw[1]);
  double scale = magnitude > p->voltageLimit ? p->voltageLimit/magnitude : 1;
  s->voltageLimited = magnitude > p->voltageLimit;
  for (int axis = 0; axis < 2; axis++) {
    applied[axis] = raw[axis]*scale;
    s->currentIntegral[axis] += dt*(p->currentKi*error[axis] +
                                  p->currentAw*(applied[axis]-raw[axis]));
  }
  FocInversePark(applied, angle, s->voltage);
}

/* Privileged true-flux wrapper, retained ONLY for the oracle benchmark. */
static inline void FocCurrentStep(FocState *s, const FocParams *p,
                                  const double current[2], const double rotorFlux[2],
                                  double speed, double dt) {
  double flux = hypot(rotorFlux[0], rotorFlux[1]);
  double angle = flux > 1e-8 ? atan2(rotorFlux[1], rotorFlux[0]) : 0;
  FocCurrentStepFrame(s, p, current, flux, angle, speed, dt);
}
#endif
