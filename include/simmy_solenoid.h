#ifndef SIMMY_SOLENOID_H
#define SIMMY_SOLENOID_H

/* Drive a linear actuator with coil voltage and couple its force to the
 * existing body chain. SimmyLinearMount and the ChainStep wrappers are
 * declared in simmy.h, after SimmyChainState.
 *
 * Reluctance: F = 0.5*I^2*dL/dx. The coordinate x is the air gap, so
 * force is always <= 0: reversing current cannot reverse attraction.
 *
 * Voice coil: F = Kf*I, with back EMF Kf*velocity. Reversing current
 * reverses force. This is the brushed-DC electrical equation in linear
 * units, without a commutator or a rotary coordinate.
 *
 * Reluctance inductance follows one position-dependent curve:
 *   L(x) = Lmin + (Lmax-Lmin)/(1+x/shapeMeters)^saturationSharpness
 * A small shapeMeters and large Lmax/Lmin give strong gap sensitivity;
 * a large shapeMeters gives weak variation over the working travel.
 * This is not current-dependent iron saturation: magnetics remain linear
 * in current. Neither model includes hysteresis, core loss, or thermal
 * dynamics beyond cumulative winding heat.
 *
 * ReluctanceStep integrates flux linkage, not current alone:
 *   I_new = (L_old*I_old + dt*V)/(L_new + dt*R)
 * Reusing L_new for the old term would preserve current instead of flux
 * when the gap moves. The stored inductance is therefore part of the
 * state, not a disposable cache. Init starts with zero flux; to seed a
 * nonzero current, also seed inductance at the initial gap.
 *
 * minGapMeters bounds electrical calculations only. It does not stop the
 * mechanical body; callers must enforce travel limits separately. The
 * force uses dL/dx evaluated at the bounded gap, even below that gap.
 *
 * Voice-coil Kf is constant over the modeled stroke. Field roll-off,
 * interactions between separate magnets, and multi-body magnetic forces
 * are outside these single-coil models. See docs/motors/solenoid.md.
 */

/* ==================== Reluctance (attraction-only) actuator ========== */

typedef struct {
  double resistance;          /* ohms, >=0 */
  double minInductance;       /* H, >0 -- L as gap->infinity */
  double maxInductance;       /* H, >minInductance -- L as gap->0 */
  double shapeMeters;         /* >0, gap scale of the inductance curve */
  double saturationSharpness; /* >0, dimensionless exponent; not current saturation */
  double minGapMeters;        /* >0, lower electrical gap bound, NOT a mechanical stop */
} SimmyReluctanceParams;

typedef struct {
  double current;       /* A */
  double inductance;    /* H, L at the last step's gap; reconstructs stored flux */
  double windingHeatJ, inputJ, mechanicalWorkJ, numericalLossJ; /* J, cumulative */
} SimmyReluctanceState;

typedef struct {
  double force;          /* N, <=0, toward smaller gap regardless of current sign */
  double inductance;     /* H, L at the bounded gap */
  double magneticEnergy; /* J, 0.5*L*I^2 */
  double windingLossW, inputPower, mechanicalPower; /* W */
  double numericalLossJ; /* J, signed energy residual; not necessarily dissipation */
} SimmyReluctanceOutput;

int SimmyReluctanceParamsValid(const SimmyReluctanceParams *params);
void SimmyReluctanceInit(SimmyReluctanceState *state);
/* Diagnostic L(x); requires valid params and finite gapMeters.
 * Clamps gapMeters to params->minGapMeters, as Step does. */
double SimmyReluctanceInductance(const SimmyReluctanceParams *params, double gapMeters);
/* Backward Euler on flux linkage, held gap/gap velocity over dt>0.
 * Returns 0 on invalid input or unrepresentable result, leaving state
 * and output unchanged; returns 1 on success. */
int SimmyReluctanceStep(SimmyReluctanceState *state, const SimmyReluctanceParams *params,
                        double voltage, double gapMeters, double gapVelocityMetersPerSec,
                        double dt, SimmyReluctanceOutput *output);

/* ==================== Voice-coil / moving-magnet actuator ============= */

typedef struct {
  double resistance;    /* ohms, >=0 */
  double inductance;    /* H, >0 */
  double forceConstant; /* N/A == V*s/m, >=0; both force and back-EMF constant */
} SimmyVoiceCoilParams;

typedef struct {
  double current; /* A */
  double windingHeatJ, inputJ, mechanicalWorkJ, numericalLossJ; /* J, cumulative */
} SimmyVoiceCoilState;

typedef struct {
  double force;          /* N, signed */
  double magneticEnergy; /* J */
  double windingLossW, inputPower; /* W */
  double numericalLossJ; /* J, backward-Euler dissipation */
} SimmyVoiceCoilOutput;

int SimmyVoiceCoilParamsValid(const SimmyVoiceCoilParams *params);
void SimmyVoiceCoilInit(SimmyVoiceCoilState *state);
/* Backward Euler on L*dI/dt = V - R*I - Kf*velocity, held velocity
 * over dt>0. Same return and failure-atomicity convention as ReluctanceStep. */
int SimmyVoiceCoilStep(SimmyVoiceCoilState *state, const SimmyVoiceCoilParams *params,
                       double voltage, double velocityMetersPerSec, double dt,
                       SimmyVoiceCoilOutput *output);

#endif /* SIMMY_SOLENOID_H */
