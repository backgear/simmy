#include "simmy.h"

#include <math.h>

#define SIMMY_STICTION_VELOCITY_EPSILON 0.05
#define SIMMY_DEFAULT_SUBSTEPS 20
#define SIMMY_TWO_PI 6.28318530717958647692

/* ---- shared tiny PRNG (xorshift32 -- deterministic, not cryptographic;
 * used for both analog-chain noise and pulse-train jitter) ----------- */
static uint32_t NextRng(uint32_t *state) {
  uint32_t x = *state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  *state = x;
  return x;
}

static double UniformSigned(uint32_t *state) {
  return (double)NextRng(state) / 4294967296.0 * 2.0 - 1.0;
}

static int32_t RoundToCounts(double v) {
  return (int32_t)(v >= 0 ? v + 0.5 : v - 0.5);
}

/* ==================== Mechanical body chain ========================= */

/* The textbook "decide before integrating" stiction formulation: checks
 * whether the driving force can overcome static friction BEFORE clamping
 * velocity to 0, not after -- avoids the classic bug where a naive
 * post-hoc threshold makes velocity chatter around zero every tick. */
static void BodyMechanicalStep(SimmyBodyState *s, const SimmyBodyParams *p,
                                double drivingForce, double dt) {
  drivingForce -= p->gravityBias;
  if (p->rippleAmplitude != 0.0 && p->ripplePeriodCounts != 0.0) {
    drivingForce -= p->rippleAmplitude * sin(SIMMY_TWO_PI * s->position / p->ripplePeriodCounts);
  }

  double stictionLimit = p->stictionLimit;
  double coulombLevel = p->coulombLevel;
  if (p->frictionBumpHalfWidth > 0.0 &&
      fabs(s->position - p->frictionBumpCenter) <= p->frictionBumpHalfWidth) {
    stictionLimit += p->frictionBumpExtra;
    coulombLevel += p->frictionBumpExtra;
  }

  if (stictionLimit > 0.0 && fabs(s->velocity) < SIMMY_STICTION_VELOCITY_EPSILON &&
      fabs(drivingForce) <= stictionLimit) {
    s->velocity = 0.0;
    return;
  }

  double frictionForce = p->friction * s->velocity;
  if (coulombLevel > 0.0) {
    double slideDirection = (fabs(s->velocity) < SIMMY_STICTION_VELOCITY_EPSILON)
        ? (drivingForce > 0.0 ? 1.0 : -1.0)
        : (s->velocity > 0.0 ? 1.0 : -1.0);
    frictionForce += coulombLevel * slideDirection;
  }
  s->velocity += (drivingForce - frictionForce) / p->inertia * dt;
  s->position += s->velocity * dt;
}

static double CouplingRelativeDisplacement(double posA, double posB, double gap) {
  double rel = posA - posB;
  if (gap <= 0.0) return rel;
  if (rel > gap) return rel - gap;
  if (rel < -gap) return rel + gap;
  return 0.0;
}

void SimmyChainInit(SimmyChainState *state, const SimmyChainParams *params) {
  (void)params;
  for (int i = 0; i < SIMMY_MAX_BODIES; i++) {
    state->bodies[i].position = 0.0;
    state->bodies[i].velocity = 0.0;
  }
}

static void ChainStep(SimmyChainState *state, const SimmyChainParams *params, double force,
                      const SimmyStepperParams *motor, double radiansPerCount,
                      double forcePerNm, double currentA, double currentB) {
  int n = params->bodyCount;
  if (n <= 1 && !motor) {
    BodyMechanicalStep(&state->bodies[0], &params->bodies[0], force, 1.0);
    return;
  }

  int subSteps = params->subSteps > 0 ? params->subSteps : SIMMY_DEFAULT_SUBSTEPS;
  double dt = 1.0 / subSteps;
  double couplingForce[SIMMY_MAX_BODIES - 1];

  for (int step = 0; step < subSteps; step++) {
    if (motor) {
      force = forcePerNm * SimmyStepperTorque(motor,
          state->bodies[0].position * radiansPerCount, currentA, currentB);
    }
    for (int i = 0; i < n - 1; i++) {
      double rel = CouplingRelativeDisplacement(state->bodies[i].position, state->bodies[i + 1].position,
                                                 params->couplings[i].backlashGap);
      double relVel = state->bodies[i].velocity - state->bodies[i + 1].velocity;
      couplingForce[i] = params->couplings[i].springK * rel + params->couplings[i].springD * relVel;
    }
    for (int i = 0; i < n; i++) {
      double netForce;
      if (n == 1) netForce = force;
      else if (i == 0) netForce = force - couplingForce[0];
      else if (i == n - 1) netForce = couplingForce[n - 2];
      else netForce = couplingForce[i - 1] - couplingForce[i];
      BodyMechanicalStep(&state->bodies[i], &params->bodies[i], netForce, dt);
    }
  }
}

void SimmyChainStep(SimmyChainState *state, const SimmyChainParams *params, double force) {
  ChainStep(state, params, force, NULL, 0, 0, 0, 0);
}

void SimmyStepperChainStep(SimmyChainState *state, const SimmyChainParams *params,
                           const SimmyStepperParams *motor, double radiansPerCount,
                           double forcePerNm, double currentA, double currentB) {
  ChainStep(state, params, 0, motor, radiansPerCount, forcePerNm, currentA, currentB);
}

void SimmyStepperCurrents(double commandAngle, double peakCurrent,
                         const SimmyStepperCurrentParams *params, double currents[2]) {
  double ideal[2] = {peakCurrent * cos(commandAngle), peakCurrent * sin(commandAngle)};
  for (int i = 0; i < 2; i++) {
    double value = ideal[i];
    if (params->quantizeStep[i] > 0)
      value = round(value / params->quantizeStep[i]) * params->quantizeStep[i];
    currents[i] = value * (1 + params->gainError[i]) + params->offset[i];
  }
}

double SimmyStepperMicrostepAngle(int64_t index, int microstepsPerFullStep) {
  int64_t cycle = 4LL * microstepsPerFullStep;
  return SIMMY_TWO_PI * (double)(index % cycle) / (double)cycle;
}

double SimmyStepperTorque(const SimmyStepperParams *params, double rotorAngle,
                          double currentA, double currentB) {
  double angle = params->electricalCyclesPerRev * rotorAngle + params->electricalOffset;
  return params->torqueConstant * (-currentA * sin(angle) + currentB * cos(angle))
      - params->detentAmplitude * sin(params->detentCyclesPerRev * rotorAngle + params->detentPhase);
}

int32_t SimmyChainFeedbackPositionCounts(const SimmyChainState *state, const SimmyChainParams *params) {
  return RoundToCounts(state->bodies[params->feedbackBody].position);
}
int32_t SimmyChainFeedbackVelocityCounts(const SimmyChainState *state, const SimmyChainParams *params) {
  return RoundToCounts(state->bodies[params->feedbackBody].velocity);
}
int32_t SimmyChainBodyPositionCounts(const SimmyChainState *state, int bodyIndex) {
  return RoundToCounts(state->bodies[bodyIndex].position);
}
int32_t SimmyChainBodyVelocityCounts(const SimmyChainState *state, int bodyIndex) {
  return RoundToCounts(state->bodies[bodyIndex].velocity);
}

/* ==================== Analog signal chain ============================ */

void SimmyAnalogChainInit(SimmyAnalogChainState *state, uint32_t noiseSeed) {
  for (int i = 0; i < SIMMY_MAX_ANALOG_STAGES; i++) state->lagState[i] = 0.0;
  state->rngState = noiseSeed ? noiseSeed : 0x9e3779b9u;
}

double SimmyAnalogChainStep(SimmyAnalogChainState *state, const SimmyAnalogChainParams *params,
                             double input) {
  double value = input;
  for (int i = 0; i < params->stageCount; i++) {
    const SimmyAnalogStageParams *stage = &params->stages[i];
    switch (stage->type) {
      case SIMMY_STAGE_GAIN:
        value *= stage->param0;
        break;
      case SIMMY_STAGE_SATURATE:
        if (value < stage->param0) value = stage->param0;
        if (value > stage->param1) value = stage->param1;
        break;
      case SIMMY_STAGE_QUANTIZE:
        if (stage->param0 > 0.0) value = round(value / stage->param0) * stage->param0;
        break;
      case SIMMY_STAGE_LAG:
        state->lagState[i] += (value - state->lagState[i]) * stage->param0;
        value = state->lagState[i];
        break;
      case SIMMY_STAGE_NOISE:
        value += stage->param0 * UniformSigned(&state->rngState);
        break;
    }
  }
  return value;
}

/* ==================== Pulse source (bidirectional) ==================== */

void SimmyPulseTrainInit(SimmyPulseTrainState *state, uint32_t seed) {
  state->phase = 0.0;
  state->rngState = seed ? seed : 0x9e3779b9u;
}

int32_t SimmyPulseTrainTickDelta(SimmyPulseTrainState *state, double rateEdgesPerSec,
                                  double tickHz, double jitterFraction) {
  double nominalIncrement = rateEdgesPerSec / tickHz;
  double increment = nominalIncrement;
  if (jitterFraction > 0.0) {
    double jitterMag = fabs(nominalIncrement) * jitterFraction;
    increment += jitterMag * UniformSigned(&state->rngState);
    /* Jitter models short-term timing noise, not a free direction
     * reversal -- never let it flip the sign of this tick's increment. */
    if (nominalIncrement >= 0.0 && increment < 0.0) increment = 0.0;
    if (nominalIncrement < 0.0 && increment > 0.0) increment = 0.0;
  }
  state->phase += increment;
  double whole = trunc(state->phase);   /* round-toward-zero -- symmetric for +/- rates,
                                            unlike floor() which would bias negative phases */
  state->phase -= whole;
  return (int32_t)whole;
}
