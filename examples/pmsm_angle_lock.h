#ifndef PMSM_ANGLE_LOCK_H
#define PMSM_ANGLE_LOCK_H

/* Open-loop "forced alignment" / angle-lock torque pulse on simmy's
 * existing sine-PM plant (SimmyPmStep). This is the continuous-angle
 * analogue of a real no-Hall drive's startup maneuver: hold the applied
 * stator voltage vector at a FIXED electrical angle -- never recomputed
 * from the rotor's live angle, unlike every FOC example in this
 * directory -- and let the permanent-magnet rotor swing into alignment
 * with it. There is no controller and no feedback here at all, by
 * design: the point is to show the PLANT's own electromechanical
 * response is an ordinary damped second-order system, so sweeping rotor
 * inertia (the one thing this file varies) should reproduce the
 * textbook period/sqrt(J) scaling. See docs/motors/bldc.md.
 *
 * SimmyPmStep still takes the rotor's own live mechanical angle/speed as
 * input, exactly as always -- that comes from closing the mechanical
 * loop with a single-body SimmyChainParams/SimmyChainState below, NOT
 * from the fixed drive angle, which only ever sets the constant
 * alpha/beta voltage.
 *
 * The restoring torque, worked out from the plant's own equations
 * (SimmyPmStep's held-current-shape torque term and back-EMF term, both
 * k(theta)=Ke*(-sin theta, cos theta)), assuming the electrical dynamics
 * are fast relative to the mechanical ones (good once J is not tiny;
 * see PmAngleLockElectricalDampingNm's doc comment for the exact
 * condition and where it starts to show its own lag):
 *
 *   torque(theta) ~= 1.5*Ke*Vm/R * sin(driveAngle-theta) - Bem*speed
 *   Bem = 1.5*Ke^2/R                              (electrical damping)
 *
 * theta = polePairs*mechanicalAngle + electricalOffset is the rotor's
 * own electrical angle. Linearized near the stable equilibrium
 * theta==driveAngle, that is exactly J*x'' + Bem*x' + K*x = 0 with
 * K = 1.5*Ke*Vm/R*polePairs -- an ordinary damped harmonic oscillator,
 * entirely from the motor's own resistance and back EMF: NO added
 * mechanical friction is needed to see realistic decaying oscillation.
 *
 * Units/mapping: SimmyBodyParams.inertia is "counts/tick^2 per unit of
 * applied force" (see include/simmy.h); this file fixes
 * mount.radiansPerCount == mount.tickSeconds == PMSM_ANGLE_LOCK_DT,
 * which makes chain.bodies[0].inertia == J_SI/PMSM_ANGLE_LOCK_DT (see
 * PmAngleLockTrialInit) and makes mechanics.bodies[0].velocity read
 * directly as mechanical rad/s with no further scaling -- the same
 * convention examples/pmsm_foc.h uses for its own mechanical body. */
#include "simmy.h"
#include <math.h>

#define PMSM_ANGLE_LOCK_DT .0001
#define PMSM_ANGLE_LOCK_PI 3.14159265358979323846

typedef struct {
  SimmyPmParams motor;
  SimmyPmState electrical;
  double voltage[2]; /* FIXED alpha/beta drive; set once, never recomputed from angle */
  SimmyChainParams chain;
  SimmyChainState mechanics;
  SimmyMotorMount mount;
} PmAngleLockTrial;

/* driveAngleElectricalRad is the fixed electrical angle the drive vector
 * points at, held in the STATOR frame for the whole run. voltageMagnitude
 * is its peak volts. inertiaKgM2 is the swept parameter.
 * frictionNmPerRadSec is EXTRA viscous friction on top of whatever
 * damping the electrical circuit itself already provides (0 is the
 * normal choice here -- see the file header); it exists only so a
 * caller can experiment, not because this file needs it. */
static inline PmAngleLockTrial PmAngleLockTrialInit(double driveAngleElectricalRad,
                                                    double voltageMagnitude,
                                                    double inertiaKgM2,
                                                    double frictionNmPerRadSec) {
  PmAngleLockTrial t = {0};
  /* Same sample motor as docs/motors/bldc.md, sine instead of trapezoidal. */
  t.motor = (SimmyPmParams){.polePairs = 4, .waveform = SIMMY_PM_SINE, .resistance = .6,
      .inductance = .002, .backEmfConstant = .08};
  SimmyPmInit(&t.electrical);
  t.voltage[0] = voltageMagnitude*cos(driveAngleElectricalRad);
  t.voltage[1] = voltageMagnitude*sin(driveAngleElectricalRad);
  t.chain.bodyCount = 1;
  t.chain.subSteps = 1;
  t.mount = (SimmyMotorMount){.radiansPerCount = PMSM_ANGLE_LOCK_DT, .forcePerNm = 1,
                              .tickSeconds = PMSM_ANGLE_LOCK_DT};
  t.chain.bodies[0].inertia = inertiaKgM2/PMSM_ANGLE_LOCK_DT;
  t.chain.bodies[0].friction = frictionNmPerRadSec;
  SimmyChainInit(&t.mechanics, &t.chain);
  return t;
}

static inline double PmAngleLockSpeed(const PmAngleLockTrial *t) {
  return t->mechanics.bodies[0].velocity*t->mount.radiansPerCount/t->mount.tickSeconds;
}

static inline double PmAngleLockAngle(const PmAngleLockTrial *t) {
  return t->mechanics.bodies[0].position*t->mount.radiansPerCount;
}

/* Sets the rotor's starting mechanical angle/speed directly -- used only
 * to pick a displaced initial condition before the run starts; not part
 * of the normal step loop. */
static inline void PmAngleLockSetState(PmAngleLockTrial *t, double mechanicalAngle, double speed) {
  t->mechanics.bodies[0].position = mechanicalAngle/t->mount.radiansPerCount;
  t->mechanics.bodies[0].velocity = speed*t->mount.tickSeconds/t->mount.radiansPerCount;
}

/* Single electrical+mechanical tick at PMSM_ANGLE_LOCK_DT. t->voltage is
 * NEVER recomputed here -- SimmyPmStep still needs the rotor's live
 * angle/speed as its own input, for correct back-EMF/torque, exactly
 * like every other simmy motor step function. */
static inline int PmAngleLockTick(PmAngleLockTrial *t, SimmyPmOutput *output) {
  double angle = PmAngleLockAngle(t), speed = PmAngleLockSpeed(t);
  if (!SimmyPmStep(&t->electrical, &t->motor, t->voltage, angle, speed, PMSM_ANGLE_LOCK_DT, output))
    return 0;
  SimmyChainStepDt(&t->mechanics, &t->chain, output->torque/t->mount.forcePerNm, 1.0);
  return 1;
}

/* The nearest stable mechanical equilibrium: where the rotor's own
 * electrical angle matches the fixed drive vector's electrical angle. */
static inline double PmAngleLockEquilibrium(double driveAngleElectricalRad,
                                            const SimmyPmParams *motor) {
  return (driveAngleElectricalRad-motor->electricalOffset)/motor->polePairs;
}

/* The electrical damping term Bem = 1.5*Ke^2/R derived in the file
 * header -- NOT a mechanical friction coefficient, but it adds exactly
 * like one in the linearized equation of motion. Valid once the
 * electrical time constant L/R is small relative to the mechanical
 * oscillation period; for this sample motor (L/R ~ 3.3 ms) that holds
 * increasingly well as J grows, and measurably less well for very small
 * J -- see tests/test_pmsm_angle_lock.c's tolerance on exactly that. */
static inline double PmAngleLockElectricalDampingNm(const SimmyPmParams *motor) {
  return 1.5*motor->backEmfConstant*motor->backEmfConstant/motor->resistance;
}
#endif
