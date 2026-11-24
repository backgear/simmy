#include "simmy.h"

#include <math.h>
#include <string.h>

/* ==================== Reluctance (attraction-only) actuator ========== */

int SimmyReluctanceParamsValid(const SimmyReluctanceParams *p) {
  return p && isfinite(p->resistance) && p->resistance >= 0 &&
      isfinite(p->minInductance) && p->minInductance > 0 &&
      isfinite(p->maxInductance) && p->maxInductance > p->minInductance &&
      isfinite(p->shapeMeters) && p->shapeMeters > 0 &&
      isfinite(p->saturationSharpness) && p->saturationSharpness > 0 &&
      isfinite(p->minGapMeters) && p->minGapMeters > 0;
}

void SimmyReluctanceInit(SimmyReluctanceState *s) { memset(s, 0, sizeof(*s)); }

double SimmyReluctanceInductance(const SimmyReluctanceParams *p, double gapMeters) {
  double x = fmax(gapMeters, p->minGapMeters);
  double u = x / p->shapeMeters;
  return p->minInductance + (p->maxInductance - p->minInductance) *
      pow(1.0 + u, -p->saturationSharpness);
}

/* dL/dx at the already-clamped gap x. Always <= 0 (L falls as the gap
 * opens), which is what makes the reluctance force below structurally
 * attractive -- see the file header comment in include/simmy_solenoid.h. */
static double ReluctanceDLdx(const SimmyReluctanceParams *p, double x) {
  double u = x / p->shapeMeters;
  return -(p->saturationSharpness * (p->maxInductance - p->minInductance) / p->shapeMeters) *
      pow(1.0 + u, -(p->saturationSharpness + 1.0));
}

int SimmyReluctanceStep(SimmyReluctanceState *s, const SimmyReluctanceParams *p,
                        double voltage, double gapMeters, double gapVelocityMetersPerSec,
                        double dt, SimmyReluctanceOutput *output) {
  if (!s || !output || !SimmyReluctanceParamsValid(p) || !isfinite(voltage) ||
      !isfinite(gapMeters) || !isfinite(gapVelocityMetersPerSec) || !isfinite(dt) || dt <= 0 ||
      !isfinite(s->current) || !isfinite(s->inductance)) {
    return 0;
  }

  double x = fmax(gapMeters, p->minGapMeters);
  double lNew = SimmyReluctanceInductance(p, x);
  double magneticEnergyOld = .5 * s->inductance * s->current * s->current;

  /* Backward Euler on flux linkage psi=L(x)*I, NOT on current directly --
   * see the product-rule note in include/simmy_solenoid.h. s->inductance
   * is L at the gap the stored current was last stepped against, so
   * s->inductance*s->current is the correct psi_old even if the gap moved
   * since the previous call. */
  double psiOld = s->inductance * s->current;
  double next = (psiOld + dt * voltage) / (lNew + dt * p->resistance);
  if (!isfinite(next)) return 0;

  double dLdx = ReluctanceDLdx(p, x);
  SimmyReluctanceOutput o = {0};
  o.inductance = lNew;
  o.force = .5 * next * next * dLdx; /* <= 0: always attractive, see header comment */
  o.magneticEnergy = .5 * lNew * next * next;
  o.windingLossW = p->resistance * next * next;
  o.inputPower = voltage * next;
  o.mechanicalPower = o.force * gapVelocityMetersPerSec;
  /* Defined as the residual that makes the energy ledger below balance
   * exactly: for constant L this collapses to SimmyDcStep's closed form
   * .5*L*(I_new-I_old)^2 (verified by hand; kept as a direct residual
   * here because L itself is changing, not just I). */
  o.numericalLossJ = dt * o.inputPower - dt * o.windingLossW - dt * o.mechanicalPower -
      (o.magneticEnergy - magneticEnergyOld);

  if (!isfinite(o.force) || !isfinite(o.magneticEnergy) || !isfinite(o.windingLossW) ||
      !isfinite(o.inputPower) || !isfinite(o.mechanicalPower) || !isfinite(o.numericalLossJ)) {
    return 0;
  }

  SimmyReluctanceState result = *s;
  result.current = next;
  result.inductance = lNew;
  result.windingHeatJ += dt * o.windingLossW;
  result.inputJ += dt * o.inputPower;
  result.mechanicalWorkJ += dt * o.mechanicalPower;
  result.numericalLossJ += o.numericalLossJ;
  if (!isfinite(result.windingHeatJ) || !isfinite(result.inputJ) ||
      !isfinite(result.mechanicalWorkJ) || !isfinite(result.numericalLossJ)) {
    return 0;
  }

  *s = result;
  *output = o;
  return 1;
}

int SimmyReluctanceChainStep(SimmyChainState *chainState, const SimmyChainParams *chain,
                             SimmyReluctanceState *motorState, const SimmyReluctanceParams *motor,
                             const SimmyLinearMount *mount, double voltage, double dtSeconds) {
  double gapMeters = chainState->bodies[0].position * mount->metersPerCount;
  double gapVelocity = chainState->bodies[0].velocity * mount->metersPerCount / mount->tickSeconds;
  SimmyReluctanceOutput output;
  if (!SimmyReluctanceStep(motorState, motor, voltage, gapMeters, gapVelocity, dtSeconds, &output)) {
    return 0;
  }
  SimmyChainStepDt(chainState, chain, mount->forcePerNewton * output.force,
                    dtSeconds / mount->tickSeconds);
  return 1;
}

/* ==================== Voice-coil / moving-magnet actuator ============= */

int SimmyVoiceCoilParamsValid(const SimmyVoiceCoilParams *p) {
  return p && isfinite(p->resistance) && p->resistance >= 0 &&
      isfinite(p->inductance) && p->inductance > 0 &&
      isfinite(p->forceConstant) && p->forceConstant >= 0;
}

void SimmyVoiceCoilInit(SimmyVoiceCoilState *s) { memset(s, 0, sizeof(*s)); }

int SimmyVoiceCoilStep(SimmyVoiceCoilState *s, const SimmyVoiceCoilParams *p,
                       double voltage, double velocityMetersPerSec, double dt,
                       SimmyVoiceCoilOutput *output) {
  if (!s || !output || !SimmyVoiceCoilParamsValid(p) || !isfinite(voltage) ||
      !isfinite(velocityMetersPerSec) || !isfinite(dt) || dt <= 0 || !isfinite(s->current)) {
    return 0;
  }
  /* Same backward-Euler RL-plus-back-EMF update as SimmyDcStep, in linear
   * units; see the file header for why this is a distinct API rather than
   * a SimmyDcParams alias. */
  double next = (p->inductance / dt * s->current + voltage - p->forceConstant * velocityMetersPerSec) /
      (p->inductance / dt + p->resistance);
  if (!isfinite(next)) return 0;

  SimmyVoiceCoilOutput o = {0};
  o.force = p->forceConstant * next; /* signed: follows current, unlike reluctance force */
  o.magneticEnergy = .5 * p->inductance * next * next;
  o.windingLossW = p->resistance * next * next;
  o.inputPower = voltage * next;
  double change = next - s->current;
  o.numericalLossJ = .5 * p->inductance * change * change;
  if (!isfinite(o.force) || !isfinite(o.magneticEnergy) || !isfinite(o.windingLossW) ||
      !isfinite(o.inputPower) || !isfinite(o.numericalLossJ)) {
    return 0;
  }

  SimmyVoiceCoilState result = *s;
  result.current = next;
  result.windingHeatJ += dt * o.windingLossW;
  result.inputJ += dt * o.inputPower;
  result.mechanicalWorkJ += dt * o.force * velocityMetersPerSec;
  result.numericalLossJ += o.numericalLossJ;
  if (!isfinite(result.windingHeatJ) || !isfinite(result.inputJ) ||
      !isfinite(result.mechanicalWorkJ) || !isfinite(result.numericalLossJ)) {
    return 0;
  }

  *s = result;
  *output = o;
  return 1;
}

int SimmyVoiceCoilChainStep(SimmyChainState *chainState, const SimmyChainParams *chain,
                            SimmyVoiceCoilState *motorState, const SimmyVoiceCoilParams *motor,
                            const SimmyLinearMount *mount, double voltage, double dtSeconds) {
  double velocity = chainState->bodies[0].velocity * mount->metersPerCount / mount->tickSeconds;
  SimmyVoiceCoilOutput output;
  if (!SimmyVoiceCoilStep(motorState, motor, voltage, velocity, dtSeconds, &output)) {
    return 0;
  }
  SimmyChainStepDt(chainState, chain, mount->forcePerNewton * output.force,
                    dtSeconds / mount->tickSeconds);
  return 1;
}
