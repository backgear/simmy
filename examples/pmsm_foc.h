#ifndef PMSM_FOC_H
#define PMSM_FOC_H

/* Example controller, NOT a new plant or production claim. Continuous-
 * angle (resolver/encoder-style) FOC for simmy's existing sine-PM plant
 * (SimmyPmParams/SimmyPmState/SimmyPmStep, waveform=SIMMY_PM_SINE), the
 * same amplitude-invariant Clarke/Park-based current+speed PI shape as
 * examples/induction_foc.h -- see that file and examples/foc_transforms.h
 * for the shared Park/inverse-Park/clamp helpers this reuses.
 *
 * Deliberately different from the induction controller in one structural
 * way: a permanent magnet's flux does not build up over time the way an
 * induction rotor's does, so there is no fluxReady gate here. Id reference
 * is fixed at 0 (maximum-torque-per-amp for the plant's non-salient model,
 * which uses one inductance for both axes -- see SimmyPmParams).
 *
 * This is purely a continuous SENSED ELECTRICAL ANGLE commutation, exactly
 * the contract SimmyBldcSixStep already documents ("angle is sensed/
 * calibrated electrical angle, not a motor-state pointer"): the caller
 * supplies that angle from whatever encoder/resolver reading its own
 * firmware uses. Simmy has NO Hall-sensor/Hall-table commutation model,
 * by design, and this controller does not add one -- it is the reference
 * harness for testing a no-Hall, FOC-commutated servo amplifier's
 * firmware (e.g. an AMC SX25A-class drive) against a simulated plant,
 * not a sensorless/back-EMF angle estimator either. */
#include "simmy.h"
#include "foc_transforms.h"
#include <math.h>

typedef struct {
  int polePairs;
  double resistance, inductance, backEmfConstant; /* same Ke used by SimmyPmParams */
  double currentLimit, voltageLimit;
  double currentKp, currentKi, currentAw;
  double speedKp, speedKi, speedAw;
} PmFocParams;

typedef struct {
  double currentIntegral[2], speedIntegral;
  double currentReference[2], measuredCurrent[2];
  double torqueRequest, voltage[2];
  int voltageLimited, currentLimited;
} PmFocState;

static inline PmFocParams PmFocDefaultParams(const SimmyPmParams *motor, double voltageLimit) {
  double bandwidth = 2*FOC_PI*300;
  double speedBandwidth = 2*FOC_PI*5, inertia = .0008; /* matches docs/motors/bldc.md's sample motor */
  PmFocParams p = {.polePairs=motor->polePairs, .resistance=motor->resistance,
      .inductance=motor->inductance, .backEmfConstant=motor->backEmfConstant,
      .currentLimit=8, .voltageLimit=voltageLimit,
      .currentKp=bandwidth*motor->inductance, .currentKi=bandwidth*motor->resistance,
      .currentAw=bandwidth, .speedKp=2*.9*speedBandwidth*inertia,
      .speedKi=speedBandwidth*speedBandwidth*inertia, .speedAw=50};
  return p;
}

static inline void PmFocInit(PmFocState *s) { *s = (PmFocState){0}; }

/* 1 kHz speed loop in the experiment below. Id reference stays 0 (MTPA
 * for this non-salient plant); only Iq comes from the speed error. Vector
 * current-reference limit is just currentLimit since Id is always 0.
 * Valid params: positive resistance/inductance/Ke, positive current/
 * voltage limit and positive gains. */
static inline void PmFocSpeedStep(PmFocState *s, const PmFocParams *p, double reference,
                                  double speed, double dt) {
  s->currentReference[0] = 0;
  double torquePerAmp = 1.5*p->backEmfConstant;
  double iqLimit = p->currentLimit;
  double error = reference-speed;
  double raw = p->speedKp*error+s->speedIntegral;
  s->torqueRequest = FocClamp(raw, torquePerAmp*iqLimit);
  s->currentReference[1] = s->torqueRequest/torquePerAmp;
  s->currentLimited = fabs(raw) > torquePerAmp*iqLimit;
  double integration = p->speedKi*error;
  if (s->voltageLimited && raw*error > 0) integration = 0;
  s->speedIntegral += dt*(integration+p->speedAw*(s->torqueRequest-raw));
}

/* 10 kHz current control. angle is the caller-supplied SENSED ELECTRICAL
 * angle (see file header) -- this core has no idea whether it came from a
 * resolver, an absolute encoder, or anything else, same as
 * SimmyBldcSixStep's sensedElectricalAngle. speed is mechanical shaft
 * speed, used only for the standard non-salient PMSM decoupling feed-
 * forward (we = polePairs*speed). PI + decoupling, voltage vector
 * projection and back-calculation anti-windup; output is held alpha/beta
 * voltage, ready to pass straight to SimmyPmStep. */
static inline void PmFocCurrentStep(PmFocState *s, const PmFocParams *p,
                                    const double current[2], double angle,
                                    double speed, double dt) {
  FocPark(current, angle, s->measuredCurrent);
  double id = s->measuredCurrent[0], iq = s->measuredCurrent[1];
  double we = p->polePairs*speed;
  /* Cross-coupling scales with ELECTRICAL speed (the dq frame's own
   * rotation rate); back EMF scales with MECHANICAL speed -- that is
   * exactly how SimmyPmParams.backEmfConstant is defined (peak back EMF
   * per mechanical rad/s), and SimmyPmStep applies it the same way. */
  double feedforward[2] = {-we*p->inductance*iq, we*p->inductance*id+speed*p->backEmfConstant};
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

/* Small, explicitly bounded stand-in for "whatever two-phase sensor a
 * no-Hall SX25A-class amp reads instead of Halls" -- NOT a claim about
 * any specific resolver/encoder IC, and it has no dynamics of its own
 * (no sample delay, quantization, or noise beyond the static errors
 * below): a caller wanting those composes its own analog chain the way
 * SimmyStepperCurrentParams' gainError/offset pair is composed elsewhere
 * in simmy. A real resolver reports sin/cos of electrical angle, not the
 * angle itself; recovering it with atan2 is exactly what a real amplifier
 * does, so that step is part of the model, not an added idealization. */
typedef struct {
  double gainError[2]; /* fractional per-channel gain error on sin/cos; 0 is ideal */
  double offsetError;  /* electrical rad, fixed mounting/calibration error */
} PmResolverParams;

static inline double PmResolverSensedAngle(double mechanicalAngle, int polePairs,
                                           double electricalOffset,
                                           const PmResolverParams *params) {
  double theta = polePairs*mechanicalAngle+electricalOffset+params->offsetError;
  double s = (1+params->gainError[0])*sin(theta);
  double c = (1+params->gainError[1])*cos(theta);
  return atan2(s, c);
}

/* Bundles the plant (sine SimmyPmParams/State), a single-body mechanical
 * load (SimmyChainParams/State, same abstraction dc_motor.c and the
 * induction examples use), the bounded resolver stand-in, and the FOC
 * controller above -- shared by examples/pmsm_foc.c and
 * tests/test_pmsm_foc.c so the drive loop is written exactly once.
 * Not a claim about any specific real machine; see docs/motors/bldc.md. */
#define PMSM_CURRENT_DT .0001
#define PMSM_CURRENT_TICKS_PER_SPEED 10

typedef struct {
  SimmyPmParams motor;
  SimmyPmState electrical;
  PmFocParams params;
  PmFocState controller;
  PmResolverParams resolver;
  SimmyChainParams chain;
  SimmyChainState mechanics;
  SimmyMotorMount mount;
  int currentTicks, voltageLimitedTicks, currentLimitedSpeedTicks;
  double maxVoltage, maxReferenceCurrent, maxActualCurrent;
} PmFocTrial;

static inline PmFocTrial PmFocTrialInit(void) {
  PmFocTrial t = {0};
  /* Same sample motor as docs/motors/bldc.md, sine instead of trapezoidal. */
  t.motor = (SimmyPmParams){.polePairs=4, .waveform=SIMMY_PM_SINE, .resistance=.6,
      .inductance=.002, .backEmfConstant=.08};
  SimmyPmInit(&t.electrical);
  double busVoltage = 24;
  t.params = PmFocDefaultParams(&t.motor, busVoltage/sqrt(3.0));
  PmFocInit(&t.controller);
  /* Bounded static resolver error: +1%/-1% channel gain mismatch and a
   * fixed 0.002 rad mounting offset -- see PmResolverSensedAngle. */
  t.resolver = (PmResolverParams){.gainError = {.01, -.01}, .offsetError = .002};
  t.chain.bodyCount = 1;
  t.chain.subSteps = 1;
  /* J=.0008 kg m^2, B=.0005 Nm/(rad/s) -- same scale as the BLDC sample motor. */
  t.mount = (SimmyMotorMount){.radiansPerCount = PMSM_CURRENT_DT, .forcePerNm = 1,
                              .tickSeconds = PMSM_CURRENT_DT};
  t.chain.bodies[0].inertia = .0008/PMSM_CURRENT_DT;
  t.chain.bodies[0].friction = .0005;
  SimmyChainInit(&t.mechanics, &t.chain);
  return t;
}

static inline double PmFocTrialSpeed(const PmFocTrial *t) {
  return t->mechanics.bodies[0].velocity*t->mount.radiansPerCount/t->mount.tickSeconds;
}

static inline double PmFocTrialAngle(const PmFocTrial *t) {
  return t->mechanics.bodies[0].position*t->mount.radiansPerCount;
}

/* Fixed 1 kHz speed / 10 kHz current ticks, same dual-rate convention as
 * examples/induction_foc_experiment.h. brakeNm models an external load at
 * the motor shaft via the mechanical chain's coulomb/static friction. */
static inline void PmFocTrialTick(PmFocTrial *t, double speedReference, double brakeNm) {
  t->chain.bodies[0].coulombLevel = t->chain.bodies[0].stictionLimit = brakeNm;
  double speed = PmFocTrialSpeed(t);
  PmFocSpeedStep(&t->controller, &t->params, speedReference, speed, .001);
  t->currentLimitedSpeedTicks += t->controller.currentLimited;
  t->maxReferenceCurrent = fmax(t->maxReferenceCurrent,
      hypot(t->controller.currentReference[0], t->controller.currentReference[1]));
  for (int tick = 0; tick < PMSM_CURRENT_TICKS_PER_SPEED; tick++) {
    double mechanicalAngle = PmFocTrialAngle(t);
    /* Caller-supplied sensed electrical angle: the only angle input the
     * controller ever sees, exactly the SimmyBldcSixStep contract. */
    double sensedAngle = PmResolverSensedAngle(mechanicalAngle, t->motor.polePairs,
                                               t->motor.electricalOffset, &t->resolver);
    speed = PmFocTrialSpeed(t);
    PmFocCurrentStep(&t->controller, &t->params, t->electrical.current, sensedAngle, speed,
                     PMSM_CURRENT_DT);
    t->voltageLimitedTicks += t->controller.voltageLimited;
    t->maxVoltage = fmax(t->maxVoltage, hypot(t->controller.voltage[0], t->controller.voltage[1]));
    SimmyPmOutput output;
    SimmyPmStep(&t->electrical, &t->motor, t->controller.voltage, mechanicalAngle, speed,
               PMSM_CURRENT_DT, &output);
    SimmyChainStepDt(&t->mechanics, &t->chain, output.torque/t->mount.forcePerNm, 1.0);
    t->maxActualCurrent = fmax(t->maxActualCurrent, hypot(t->electrical.current[0], t->electrical.current[1]));
    t->currentTicks++;
  }
}

static inline double PmFocComparisonReference(double time) {
  return fmin(120, fmax(0, (time-.1)*150)); /* 150 rad/s^2 ramp to 120 rad/s */
}

static inline double PmFocComparisonBrake(double time) {
  return time >= 1.5 && time < 3.5 ? .3 : 0;
}

/* Deliberately above the ~170 rad/s voltage-limited ceiling this sample
 * motor/bus combination can sustain, to exercise saturation/anti-windup;
 * then a reachable request to exercise recovery. */
static inline double PmFocStressReference(double time) {
  return time < .1 ? 0 : (time < 2 ? fmin(300, (time-.1)*200) : 100);
}
#endif
