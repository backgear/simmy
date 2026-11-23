#ifndef SIMMY_H
#define SIMMY_H

#include <stddef.h>
#include <stdint.h>
#include "simmy_bldc.h"

/*
 * simmy -- a small mechatronics simulation library.
 *
 * Grew out of a servo-firmware project's native unit tests, where it
 * started as a couple of header-only toy models used to ask "does the
 * control loop track a synthetic pulse train / an accelerating one /
 * one with injected jitter." It has no idea what firmware it came from
 * and no dependency on any particular controller: it simulates a serial
 * chain of rigid bodies connected by compliant (spring+damper+backlash)
 * couplings, a composable "analog signal chain" standing in for a real
 * DAC/amplifier signal path, and a bidirectional pulse source for
 * step/dir-style position commands. Separate motor components model
 * two-phase stepper torque, an averaged voltage-driven induction machine,
 * and permanent-magnet winding/bridge circuits (including a reduced model).
 *
 * Scope and non-goals:
 *   - This is a MECHANICAL + ANALOG-SIGNAL-PATH + SIMPLIFIED MOTOR simulator. It has no
 *     opinion about control algorithms. You drive it with a plain
 *     function pointer (SimmyControllerFn) -- see below -- which may be
 *     a three-line demo P loop or a real production control cascade
 *     linked in from somewhere else. Both are ordinary implementations
 *     of the same typedef; there is no special "simple" calling
 *     convention and no separate "real" one.
 *   - Closing the control loop is assumed to happen against ONE body in
 *     the chain -- SimmyChainParams.feedbackBody -- intended to be the
 *     body a real motor-mounted encoder would read. Reading position or
 *     velocity off any OTHER body (e.g. a load/table at the far end of
 *     a lead screw, standing in for where a linear scale would sit) is
 *     fully supported for MEASUREMENT/evaluation -- precisely so you can
 *     judge whether motor-side-only control tracks acceptably at the
 *     load, which is the kind of comparison that matters for judging
 *     whether a config actually fits a real physical setup. Simmy does
 *     not support closing the loop against a non-feedback body; that is
 *     a deliberate scope boundary, not a missing feature.
 *   - Every numeric model here is intentionally simplified. The mechanical
 *     body's sinusoidal ripple (SimmyBodyParams.rippleAmplitude/
 *     ripplePeriodCounts) is just an additive stand-in for periodic torque,
 *     NOT an electrical commutation model. Dedicated motor components
 *     instead compute torque from phase currents and rotor state; avoid
 *     double-counting the same effect with extra mechanical ripple.
 *     Likewise the stiction model is the textbook "decide before
 *     integrating whether the driving force can overcome static
 *     friction" formulation -- it avoids the classic bug where a naive
 *     post-hoc check makes velocity chatter around zero, but it is not
 *     a claim about any particular real friction surface.
 *   - Nothing here is a claim about any specific real machine. Example
 *     configs (JSON files under examples/configs) are representative
 *     order-of-magnitude ratios for recognizable mechanisms, not
 *     measurements.
 *
 * Units are deliberately abstract: "counts" (an encoder-count-like
 * position unit), "counts/tick" (velocity), and a tick is nominally
 * 1ms/1kHz, matching the servo-loop rate this library was built to test
 * against -- but nothing here enforces that; tickHz is always an
 * explicit parameter. Motor electrical APIs instead use SI units, with
 * explicit conversion to the chain's original counts/tick units.
 */

#define SIMMY_MAX_BODIES 8
#define SIMMY_MAX_ANALOG_STAGES 8

/* ==================== Mechanical body chain ========================= */

/* Purely mechanical per-body properties. Every field is additive/optional
 * -- 0 disables that effect for this body. There is no electrical/
 * amplifier-lag field here; see SimmyAnalogChainParams for that. */
typedef struct {
  double inertia;              /* counts/tick^2 per unit of applied force. Required (>0) --
                                    there is no sane default for "how heavy is this body." */
  double friction;              /* viscous (velocity-proportional) drag. 0 disables. */
  double stictionLimit;         /* max static-friction holding force before breakaway.
                                    0 disables stiction (falls back to viscous-only). */
  double coulombLevel;          /* kinetic/Coulomb friction magnitude while sliding,
                                    independent of speed. 0 disables. */
  double gravityBias;           /* constant force subtracted every tick (a vertical-axis-
                                    style load). 0 disables. */
  double rippleAmplitude;       /* generic periodic (position-dependent) torque disturbance --
                                    stands in for magnetic cogging OR commutation ripple,
                                    caller's choice of period/amplitude picks which. 0 disables. */
  double ripplePeriodCounts;    /* spatial period of the ripple, in counts. Ignored if
                                    rippleAmplitude is 0. */
  double frictionBumpCenter;    /* position (counts) of a ONE-OFF localized patch of extra
                                    friction -- e.g. a dusty/less-greased section of a way,
                                    NOT a recurring effect like the ripple above. */
  double frictionBumpHalfWidth; /* counts -- half-width of the affected window. 0 disables
                                    the bump entirely. */
  double frictionBumpExtra;     /* added to BOTH stictionLimit and coulombLevel while position
                                    is within the bump window. Works even starting from 0 base
                                    friction (a patch of stiction on an otherwise-free axis). */
} SimmyBodyParams;

typedef struct {
  double position;   /* true continuous state, counts -- NOT yet quantized the way reading an
                         actual encoder would be; see SimmyChainBodyPositionCounts(). */
  double velocity;    /* counts/tick */
} SimmyBodyState;

/* A compliant connection between two ADJACENT bodies in the chain. */
typedef struct {
  double springK;      /* restoring force per count of relative displacement */
  double springD;      /* damping force per count/tick of relative velocity */
  double backlashGap;   /* counts -- half-width of the dead zone (centered on zero relative
                            displacement) where no force transmits through this coupling.
                            0 disables backlash (a plain spring+damper). */
  double forceLimit;    /* max magnitude of force this coupling can transmit (springK*rel +
                            springD*relVel, clamped to +-forceLimit). 0 disables (unlimited,
                            the previous behavior) -- this is for a friction drive (a V/poly-V
                            belt, a friction clutch) that slips once demand exceeds what
                            tension/friction or clamping force can hold; see docs/couplings.
                            Clamping only bounds the transmitted force -- it does not add a
                            slip-heat or wear ledger, and it is a constant ceiling, not a
                            speed-dependent one (a centrifugal clutch needs the latter). */
} SimmyCouplingParams;

typedef struct {
  int bodyCount;                                     /* 1..SIMMY_MAX_BODIES */
  SimmyBodyParams bodies[SIMMY_MAX_BODIES];
  SimmyCouplingParams couplings[SIMMY_MAX_BODIES - 1]; /* couplings[i] connects bodies[i] and
                                                           bodies[i+1]; only the first
                                                           (bodyCount-1) entries are used */
  int feedbackBody;    /* index into bodies[] that a controller may legitimately read back --
                           see the file header comment: intended to be the motor-mounted body.
                           Any other body remains readable for measurement, just not as a
                           control input. */
  int subSteps;        /* sub-steps per call to SimmyChainStep, for numerical stability against
                           a stiff coupling. 0 -> a built-in default (20). Irrelevant/unused
                           when bodyCount == 1 (no coupling to be unstable). */
} SimmyChainParams;

typedef struct {
  SimmyBodyState bodies[SIMMY_MAX_BODIES];
} SimmyChainState;

void SimmyChainInit(SimmyChainState *state, const SimmyChainParams *params);

/* Advances the whole chain by one tick. `force` is the PHYSICAL force
 * applied to bodies[0] -- i.e. already past any amplifier/DAC modeling
 * (see SimmyAnalogChainStep below), not a raw digital command. A
 * bodyCount==1 chain applies it directly (today's single-mass model,
 * exactly what a rigid coupling collapses to -- no coupling math, no
 * numerical stiffness risk); bodyCount>1 sub-steps the spring/damper
 * dynamics internally. */
void SimmyChainStep(SimmyChainState *state, const SimmyChainParams *params, double force);

/* Explicit duration in the chain's ORIGINAL tick units (>0). State velocity
 * remains counts/original-tick; do not rescale parameters when refining dt.
 * Held force, same mechanical substep policy as SimmyChainStep. Duration 1
 * is identical to SimmyChainStep. No wall-clock or command-rate dependency. */
void SimmyChainStepDt(SimmyChainState *state, const SimmyChainParams *params,
                       double force, double dtTicks);

/* The feedbackBody's state, rounded to the nearest whole count -- what a
 * real encoder reading of that body's continuous state would be. */
int32_t SimmyChainFeedbackPositionCounts(const SimmyChainState *state, const SimmyChainParams *params);
int32_t SimmyChainFeedbackVelocityCounts(const SimmyChainState *state, const SimmyChainParams *params);

/* Any body by index, for measurement/evaluation -- NOT meant to be fed
 * back into a controller (see the file header comment). */
int32_t SimmyChainBodyPositionCounts(const SimmyChainState *state, int bodyIndex);
int32_t SimmyChainBodyVelocityCounts(const SimmyChainState *state, int bodyIndex);

/* ==================== Two-phase stepper ============================== */

/* Low-speed current-driven hybrid stepper. All angles are radians.
 * Positive torque: Kt*(-Ia*sin(thetaE) + Ib*cos(thetaE)). Thus currents
 * (I*cos(phi), I*sin(phi)) attract the rotor toward thetaE == phi.
 * No winding dynamics, saturation, or phase-torque harmonics yet. */
typedef struct {
  int electricalCyclesPerRev;   /* e.g. 50 for a 200-full-step/rev motor; >0 */
  double electricalOffset;      /* thetaE = cycles*mechanicalAngle + offset */
  double torqueConstant;        /* Nm/A; vector current amplitude, NOT per-phase RMS */
  double detentAmplitude;       /* Nm; 0 disables */
  int detentCyclesPerRev;       /* independently configurable; >0 if enabled */
  double detentPhase;           /* torque = -amplitude*sin(cycles*angle + phase) */
} SimmyStepperParams;

typedef struct {
  double gainError[2];          /* fractional: 0 is ideal, .02 is +2% */
  double offset[2];             /* A, added after gain */
  double quantizeStep[2];       /* A/LSB, 0 disables; quantize BEFORE gain/offset */
} SimmyStepperCurrentParams;

/* Pure functions: no hidden command or rotor state. Current errors model
 * static DAC/current-regulator imperfections; optional dynamics can be
 * composed afterward using two independent analog chains. */
void SimmyStepperCurrents(double commandAngle, double peakCurrent,
                         const SimmyStepperCurrentParams *params, double currents[2]);
/* Signed step/dir index, M microsteps/full step (>0). Returns wrapped
 * electrical angle in (-2*pi, 2*pi); wrapping preserves reversal. */
double SimmyStepperMicrostepAngle(int64_t index, int microstepsPerFullStep);
double SimmyStepperTorque(const SimmyStepperParams *params, double rotorAngle,
                          double currentA, double currentB);

/* Advance one chain tick with phase currents held constant, reevaluating
 * torque from body 0 position at EVERY substep, including a one-body chain.
 * Uses params->subSteps (0 => 20). radiansPerCount and forcePerNm must be
 * positive and explicitly map the chain's abstract units to motor units.
 * Does not add electrical dynamics or modify any command/current state.
 * Disable body-0 ripple if it would double-count the same detent torque. */
void SimmyStepperChainStep(SimmyChainState *state, const SimmyChainParams *params,
                           const SimmyStepperParams *motor, double radiansPerCount,
                           double forcePerNm, double currentA, double currentB);

/* ==================== Averaged induction motor + V/Hz drive ========== */

/* Balanced squirrel-cage machine, rotor parameters referred to stator.
 * Stationary amplitude-invariant Clarke alpha/beta coordinates: phase A
 * equals alpha, balanced phase peak equals vector magnitude. SI units.
 * Unsaturated linear magnetics; no PWM, core loss, thermal or skin effects. */
typedef struct {
  int polePairs;                     /* >0: two pole pairs = four poles */
  double statorResistance, rotorResistance; /* ohms, >=0 */
  double statorLeakageInductance, rotorLeakageInductance; /* H, >0 */
  double magnetizingInductance;       /* H, >0; Ls=Lls+Lm, Lr=Llr+Lm */
} SimmyInductionParams;

typedef struct {
  double statorFlux[2], rotorFlux[2]; /* alpha/beta flux linkages, Wb turns */
} SimmyInductionState;

typedef struct {
  double statorCurrent[2], rotorCurrent[2]; /* A, rotor referred to stator */
  double torque;                     /* Nm, positive for positive rotation */
  double magneticEnergy;             /* J, includes 3/2 Clarke factor */
} SimmyInductionOutput;

int SimmyInductionParamsValid(const SimmyInductionParams *params);
void SimmyInductionInit(SimmyInductionState *state);
void SimmyInductionEvaluate(const SimmyInductionState *state,
                            const SimmyInductionParams *params, SimmyInductionOutput *output);
/* RK4 electrical integration over dtSeconds>0. Voltage and mechanical
 * speed held over this integration interval. Returns RK-weighted average
 * torque for mechanical coupling; Evaluate gives endpoint instantaneous
 * torque. Caller supplies valid params and resolves electrical time scales. */
double SimmyInductionStep(SimmyInductionState *state, const SimmyInductionParams *params,
                          double voltageAlpha, double voltageBeta,
                          double mechanicalSpeedRadSec, double dtSeconds);

typedef struct {
  double radiansPerCount, forcePerNm, tickSeconds; /* all >0, original chain units */
} SimmyMotorMount;

/* Couple to body 0 without rotor-speed feedback to the DRIVE. Machine
 * physics uses body-0 speed internally. Average electrical torque drives
 * mechanics over dt/tickSeconds; speed held within electrical RK4 stages
 * makes this a partitioned (not fully coupled RK4) solver. Refine dt with
 * drive update schedule FIXED to check coupled convergence. */
void SimmyInductionChainStep(SimmyInductionState *motorState, const SimmyInductionParams *motor,
                             SimmyChainState *chainState, const SimmyChainParams *chain,
                             const SimmyMotorMount *mount, double voltageAlpha,
                             double voltageBeta, double dtSeconds);

typedef struct {
  double baseFrequencyHz;             /* >0 */
  double basePhasePeakV;              /* >0; NOT line-line RMS */
  double maxPhasePeakV;               /* >0, averaged inverter voltage ceiling */
  double boostPhasePeakV;             /* 0..basePhasePeakV; low-speed intercept */
  double rampHzPerSecond;             /* >=0; 0 means instantaneous frequency command */
} SimmyVfdParams;

typedef struct {
  double frequencyHz, phaseRadians, phasePeakV;
} SimmyVfdState;

void SimmyVfdInit(SimmyVfdState *state);
/* Called ONLY at the chosen drive-command update rate, independent of
 * integration dt. Signed target frequency; no rotor measurements. V/Hz
 * interpolates boost..base below base Hz, continues above, then clamps.
 * At exactly zero Hz voltage is zero (no DC injection/holding mode).
 * Zero terminal voltage is NOT an open-circuit/disconnected stator. */
void SimmyVfdCommandStep(SimmyVfdState *state, const SimmyVfdParams *params,
                         double targetFrequencyHz, double commandDtSeconds);
/* Advance continuous oscillator with held frequency/amplitude. Returns
 * midpoint alpha/beta voltage for the integration interval; phase is
 * wrapped. Does NOT advance the frequency ramp or command-update clock. */
void SimmyVfdVoltageStep(SimmyVfdState *state, double dtSeconds, double voltage[2]);

/* ==================== Analog signal chain ============================ */

/* A composable "train of analog type values" standing in for a real
 * DAC -> amplifier -> motor signal path: each stage transforms a single
 * scalar value, stages run in array order. Nothing requires using all
 * of these, or in any particular order -- compose whatever path a given
 * drive's own signal chain actually has. */
typedef enum {
  SIMMY_STAGE_GAIN,       /* value *= param0 */
  SIMMY_STAGE_SATURATE,   /* clamp value to [param0, param1] */
  SIMMY_STAGE_QUANTIZE,   /* round value to the nearest multiple of param0 (a DAC's LSB step) */
  SIMMY_STAGE_LAG,        /* one-pole lag, alpha = param0 (0 < alpha <= 1; 1.0 = instant) */
  SIMMY_STAGE_NOISE,      /* add bounded pseudorandom noise, uniform in [-param0, param0] */
} SimmyAnalogStageType;

typedef struct {
  SimmyAnalogStageType type;
  double param0, param1;   /* meaning depends on type -- see the enum comments above */
} SimmyAnalogStageParams;

typedef struct {
  int stageCount;
  SimmyAnalogStageParams stages[SIMMY_MAX_ANALOG_STAGES];
} SimmyAnalogChainParams;

typedef struct {
  double lagState[SIMMY_MAX_ANALOG_STAGES];   /* only entries for LAG stages are meaningful */
  uint32_t rngState;                           /* for NOISE stages */
} SimmyAnalogChainState;

void SimmyAnalogChainInit(SimmyAnalogChainState *state, uint32_t noiseSeed);
double SimmyAnalogChainStep(SimmyAnalogChainState *state, const SimmyAnalogChainParams *params,
                             double input);

/* ==================== Controller convention =========================== */

/* The one public way to drive a simulation with a controller. Deliberately
 * shaped to match a typical fixed-point position-control tick function
 * (position target in, position/velocity feedback in, a tick duration,
 * a torque/force command out) closely enough that wrapping a REAL
 * controller is a direct passthrough, not an adapter fighting this API.
 * A three-line demo P loop and a production control cascade linked in
 * from elsewhere are both just ordinary implementations of this typedef
 * -- there is no separate "simple" calling convention. */
typedef int32_t (*SimmyControllerFn)(void *userData, int32_t targetPosition,
                                      int32_t actualPosition, int32_t actualVelocity,
                                      uint32_t dtCycles);

/* ==================== Pulse source (bidirectional) ==================== */

/* Models the quantized edge-count result a real step/dir (or quadrature)
 * pulse train would produce each tick for an idealized periodic source
 * -- NOT a claim about real electrical edge timing. rateEdgesPerSec may
 * be NEGATIVE (a reversing move); jitter is clamped so it can never flip
 * the sign of a tick's increment -- it models short-term timing noise,
 * not a free direction reversal. */
typedef struct {
  double phase;        /* accumulated fractional edge count, kept in (-1, 1) between ticks */
  uint32_t rngState;
} SimmyPulseTrainState;

void SimmyPulseTrainInit(SimmyPulseTrainState *state, uint32_t seed);
int32_t SimmyPulseTrainTickDelta(SimmyPulseTrainState *state, double rateEdgesPerSec,
                                  double tickHz, double jitterFraction);

/* ==================== JSON config loading ============================= */

/* Hand-rolled, scoped to exactly the shape simmy's own config files use
 * (nested objects of numbers/strings/arrays) -- not a general-purpose,
 * hardened JSON parser. See docs/build.md for the config file schema.
 *
 * All of these return 0 on success. On failure they return nonzero and,
 * if errMsg is non-NULL, write a human-readable message (truncated to
 * fit errMsgLen, always NUL-terminated). Each reads its own top-level
 * section from the file passed in, so one JSON file can describe a
 * complete scenario: mechanics, analog chain, and motor together. */
int SimmyChainParamsFromJsonFile(const char *path, SimmyChainParams *out,
                                  char *errMsg, size_t errMsgLen);
int SimmyAnalogChainParamsFromJsonFile(const char *path, SimmyAnalogChainParams *out,
                                        char *errMsg, size_t errMsgLen);
/* Reads "stepperMotor". electricalCyclesPerRev is required (>0, no sane
 * default); detentCyclesPerRev is required only if detentAmplitude != 0. */
int SimmyStepperParamsFromJsonFile(const char *path, SimmyStepperParams *out,
                                    char *errMsg, size_t errMsgLen);
/* Reads "stepperCurrentPath". Every field defaults to 0 (the ideal case). */
int SimmyStepperCurrentParamsFromJsonFile(const char *path, SimmyStepperCurrentParams *out,
                                           char *errMsg, size_t errMsgLen);
/* Reads "inductionMotor"; validated with SimmyInductionParamsValid. */
int SimmyInductionParamsFromJsonFile(const char *path, SimmyInductionParams *out,
                                      char *errMsg, size_t errMsgLen);
/* Reads "bldcMotor" ("connection": "wye"|"delta", "waveform": "sine"|
 * "trapezoid"); validated with SimmyBldcParamsValid. Fault-injection
 * fields (openWinding) are examples' territory, not config -- they stay
 * at their zero/disabled default. */
int SimmyBldcParamsFromJsonFile(const char *path, SimmyBldcParams *out,
                                 char *errMsg, size_t errMsgLen);
/* Reads "bldcBridge"; validated with SimmyBridgeParamsValid. openSwitch
 * likewise stays at its zero/disabled default -- it's a fault toggle. */
int SimmyBridgeParamsFromJsonFile(const char *path, SimmyBridgeParams *out,
                                   char *errMsg, size_t errMsgLen);
/* Reads "pmMotor" (the reduced two-current PM model, same string values
 * as bldcMotor's "waveform"); validated with SimmyPmParamsValid. */
int SimmyPmParamsFromJsonFile(const char *path, SimmyPmParams *out,
                               char *errMsg, size_t errMsgLen);
/* Reads "dcMotor" (brushed DC); validated with SimmyDcParamsValid. */
int SimmyDcParamsFromJsonFile(const char *path, SimmyDcParams *out,
                               char *errMsg, size_t errMsgLen);

#endif /* SIMMY_H */
