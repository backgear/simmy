#ifndef INDUCTION_IFOC_H
#define INDUCTION_IFOC_H

/* Encoder-assisted INDIRECT FOC: scalar rotor-flux current model + slip
 * calculator, no access to true motor flux. This is still a model-based
 * estimate, not a physical flux measurement or sensorless control.
 * Currents/speed below are ideal measurements for this first experiment. */
#include "induction_foc.h"

typedef struct {
  double statorCurrent[2]; /* measured stationary alpha/beta, A */
  double shaftSpeed;       /* measured/estimated encoder speed, rad/s */
} IfocMeasurements;

typedef struct {
  double fluxMagnitude, fluxAngle, slipRadSec;
} IfocEstimate;

static inline void IfocInit(IfocEstimate *estimate) { *estimate = (IfocEstimate){0}; }

/* Same-timestamp estimate is used by ALL control paths (gating, torque
 * conversion, Park/inverse Park and decoupling). Then predict it to the
 * next current tick from current/speed measurements only. No true-flux
 * argument, no SimmyInductionState/plant pointer, no live parameter copy.
 * Controller parameters are explicitly supplied nominal assumptions.
 * Uses measured dq current, not its reference, so clipping/poor tracking
 * does not masquerade as delivered magnetization/torque current. */
static inline void IfocEstimateStep(IfocEstimate *estimate, const FocParams *nominal,
                                    const IfocMeasurements *measured, double dt) {
  double dq[2];
  FocPark(measured->statorCurrent, estimate->fluxAngle, dq);
  double decay = exp(-dt/nominal->rotorTimeConstant);
  double target = nominal->lm*dq[0];
  double nextFlux = fmax(0, target + (estimate->fluxMagnitude-target)*decay);
  double midpointFlux = .5*(estimate->fluxMagnitude+nextFlux);
  estimate->slipRadSec = midpointFlux > .1*nominal->fluxReference ?
      nominal->lm/nominal->rotorTimeConstant*dq[1]/midpointFlux : 0;
  estimate->fluxAngle = remainder(estimate->fluxAngle + dt*(
      nominal->polePairs*measured->shaftSpeed+estimate->slipRadSec), 2*FOC_PI);
  estimate->fluxMagnitude = nextFlux;
}

static inline void IfocSpeedStep(FocState *control, const IfocEstimate *estimate,
                                 const FocParams *nominal, const IfocMeasurements *measured,
                                 double reference, double dt) {
  FocSpeedStep(control, nominal, reference, measured->shaftSpeed, estimate->fluxMagnitude, dt);
}

static inline void IfocCurrentStep(FocState *control, IfocEstimate *estimate,
                                   const FocParams *nominal, const IfocMeasurements *measured,
                                   double dt) {
  FocCurrentStepFrame(control, nominal, measured->statorCurrent,
                     estimate->fluxMagnitude, estimate->fluxAngle, measured->shaftSpeed, dt);
  IfocEstimateStep(estimate, nominal, measured, dt);
}
#endif
