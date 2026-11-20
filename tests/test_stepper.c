#include "simmy.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846
static const SimmyStepperParams ideal = {
  .electricalCyclesPerRev = 50, .torqueConstant = .4
};
static void near(double a, double b, double tol) { assert(fabs(a-b) < tol); }

static void torque_and_currents(void) {
  SimmyStepperCurrentParams path = {0};
  for (int i = -100; i <= 100; i++) {
    double phi = i * .19, theta = i * .013;
    double currents[2];
    SimmyStepperCurrents(phi, 1.3, &path, currents);
    near(hypot(currents[0], currents[1]), 1.3, 1e-14);
    near(SimmyStepperTorque(&ideal, theta, currents[0], currents[1]),
         .4 * 1.3 * sin(phi - 50*theta), 1e-14);
    near(SimmyStepperTorque(&ideal, theta + 2*PI/50, currents[0], currents[1]),
         SimmyStepperTorque(&ideal, theta, currents[0], currents[1]), 1e-13);
  }
  near(SimmyStepperTorque(&ideal, 0, 0, 1), .4, 1e-15);
  near(SimmyStepperTorque(&ideal, 0, 1, 0), 0, 1e-15);
  /* Stable alignment: positive displacement produces negative torque. */
  assert(SimmyStepperTorque(&ideal, .001, 1, 0) < 0);
  SimmyStepperParams shifted = ideal;
  shifted.electricalOffset = .3;
  near(SimmyStepperTorque(&shifted, 0, 1, 0), -.4*sin(.3), 1e-15);

  path = (SimmyStepperCurrentParams){.gainError={.1,-.2}, .offset={.03,-.04},
                                        .quantizeStep={.25,.125}};
  double currents[2], phi = .37;
  SimmyStepperCurrents(phi, 1, &path, currents);
  near(currents[0], round(cos(phi)/.25)*.25*1.1+.03, 1e-15);
  near(currents[1], round(sin(phi)/.125)*.125*.8-.04, 1e-15);
  SimmyStepperCurrents(phi, 0, &path, currents);
  near(currents[0], .03, 1e-15); /* offsets persist at zero command */
  near(currents[1], -.04, 1e-15);
}

static void detent_and_steps(void) {
  SimmyStepperParams motor = ideal;
  motor.detentAmplitude = .02;
  motor.detentCyclesPerRev = 200;
  motor.detentPhase = .2;
  double theta = .007;
  near(SimmyStepperTorque(&motor, theta, 0, 0), -.02*sin(200*theta+.2), 1e-15);
  near(SimmyStepperTorque(&motor, theta+2*PI/200, 0, 0),
       SimmyStepperTorque(&motor, theta, 0, 0), 1e-15);
  for (int m = 1; m <= 256; m *= 4) {
    near(SimmyStepperMicrostepAngle(1, m), PI/(2*m), 1e-15);
    near(SimmyStepperMicrostepAngle(-1, m), -PI/(2*m), 1e-15);
    near(SimmyStepperMicrostepAngle(4*m, m), 0, 1e-15);
    near(SimmyStepperMicrostepAngle(INT64_MAX, m),
         SimmyStepperMicrostepAngle(INT64_MAX % (4*m), m), 1e-15);
  }
  /* Pulse source advances a signed index, not the rotor. Exact reversal. */
  SimmyPulseTrainState pulse;
  SimmyPulseTrainInit(&pulse, 1);
  int64_t index = 0;
  /* Binary-exact increment avoids a boundary count lost to FP rounding. */
  for (int i = 0; i < 1024; i++) index += SimmyPulseTrainTickDelta(&pulse, 125, 1024, 0);
  assert(index == 125);
  for (int i = 0; i < 1024; i++) index += SimmyPulseTrainTickDelta(&pulse, -125, 1024, 0);
  assert(index == 0);
}

/* SI example mapped into counts/tick: x=C*theta, v=C*dt*omega,
 * inertia=J/(C*dt^2), friction=B/(C*dt), force unit=Nm. */
static SimmyChainParams mechanics(int substeps) {
  SimmyChainParams p = {.bodyCount=1, .subSteps=substeps};
  p.bodies[0].inertia = .8; /* J=8e-5, C=1e4, dt=1e-4 */
  p.bodies[0].friction = .004;
  return p;
}

static void equilibrium_and_friction(void) {
  SimmyChainParams p = mechanics(20);
  p.bodies[0].gravityBias = .1;
  SimmyChainState s;
  SimmyChainInit(&s, &p);
  double phi = .6;
  for (int i = 0; i < 20000; i++)
    SimmyStepperChainStep(&s, &p, &ideal, 1e-4, 1, cos(phi), sin(phi));
  near(s.bodies[0].position*1e-4, (phi-asin(.1/.4))/50, 1e-10);
  near(s.bodies[0].velocity, 0, 1e-9);
  p = mechanics(20);
  p.bodies[0].stictionLimit = .1;
  SimmyChainInit(&s, &p);
  for (int i = 0; i < 100; i++)
    SimmyStepperChainStep(&s, &p, &ideal, 1e-4, 1, cos(.1), sin(.1));
  assert(s.bodies[0].position == 0 && s.bodies[0].velocity == 0);
  SimmyStepperChainStep(&s, &p, &ideal, 1e-4, 1, 0, 1);
  assert(s.bodies[0].velocity > 0);
}

static double response(int substeps, int bodies) {
  SimmyChainParams p = mechanics(substeps);
  p.bodyCount = bodies;
  p.bodies[1] = p.bodies[0];
  p.couplings[0].springK = .001;
  p.couplings[0].springD = .01;
  SimmyChainState s;
  SimmyChainInit(&s, &p);
  for (int i = 0; i < 150; i++)
    SimmyStepperChainStep(&s, &p, &ideal, 1e-4, 1, 0, 1);
  return s.bodies[0].position;
}

static void integration(void) {
  for (int bodies = 1; bodies <= 2; bodies++) {
    double a = response(10, bodies), b = response(20, bodies), c = response(40, bodies);
    assert(fabs(c-b) < fabs(b-a));
    assert(fabs(c-b) < .1); /* counts; 1e-5 rad */
  }
  /* Explicit conversion factors: a constant initial torque, one substep. */
  SimmyChainParams p = {.bodyCount=1, .subSteps=1};
  p.bodies[0].inertia = 2;
  SimmyChainState s;
  SimmyChainInit(&s, &p);
  s.bodies[0].position = 3;
  double torque = SimmyStepperTorque(&ideal, 3*.002, 0, 1);
  SimmyStepperChainStep(&s, &p, &ideal, .002, 7, 0, 1);
  near(s.bodies[0].velocity, torque*7/2, 1e-15);
  near(s.bodies[0].position, 3+torque*7/2, 1e-15);

  /* Independent two-substep reference: the second torque MUST use the
   * updated rotor position, not a force sampled once per outer tick. */
  p.subSteps = 2;
  SimmyChainInit(&s, &p);
  s.bodies[0].position = 3;
  double x = 3, v = 0;
  for (int i = 0; i < 2; i++) {
    v += 7*.4*cos(50*x*.002)/2*.5;
    x += v*.5;
  }
  SimmyStepperChainStep(&s, &p, &ideal, .002, 7, 0, 1);
  near(s.bodies[0].position, x, 1e-15);
  near(s.bodies[0].velocity, v, 1e-15);

  /* A full-step command excites magnetic spring ringing, not a position
   * snap. It overshoots alignment and subsequently reverses velocity. */
  p = mechanics(20);
  SimmyChainInit(&s, &p);
  double maximum = 0;
  int reversed = 0;
  for (int i = 0; i < 2000; i++) {
    SimmyStepperChainStep(&s, &p, &ideal, 1e-4, 1, 0, 1);
    if (s.bodies[0].position > maximum) maximum = s.bodies[0].position;
    if (s.bodies[0].velocity < 0) reversed = 1;
  }
  assert(maximum > 1.2*(PI/2)/50/1e-4);
  assert(reversed);
}

static double velocity_error(int hz, int microsteps, int detent, int errors) {
  SimmyChainParams p = {.bodyCount=1, .subSteps=20};
  p.bodies[0].inertia = 4e-4*hz*hz/1e4;
  p.bodies[0].friction = .008*hz/1e4;
  p.bodies[0].gravityBias = .03;
  SimmyStepperParams motor = ideal;
  motor.detentAmplitude = detent ? .012 : 0;
  motor.detentCyclesPerRev = 200;
  motor.detentPhase = .3;
  SimmyStepperCurrentParams path = {0};
  if (errors)
    path = (SimmyStepperCurrentParams){.gainError={.03,-.03}, .offset={.01,-.015},
                                       .quantizeStep={1.0/31,1.0/31}};
  SimmyChainState s;
  SimmyChainInit(&s, &p);
  s.bodies[0].position = -asin((.03+.008*.2)/.4)/50*1e4;
  s.bodies[0].velocity = .2*1e4/hz;
  SimmyPulseTrainState pulse;
  SimmyPulseTrainInit(&pulse, 1);
  int64_t index = 0;
  double sum = 0;
  for (int i = 0; i < 4*hz; i++) {
    double phi = microsteps ? SimmyStepperMicrostepAngle(index, microsteps) : 10.0*i/hz;
    double currents[2];
    SimmyStepperCurrents(phi, 1, &path, currents);
    if (i >= 2*hz) {
      double error = s.bodies[0].velocity*hz/1e4/.2 - 1;
      sum += error*error;
    }
    SimmyStepperChainStep(&s, &p, &motor, 1e-4, 1, currents[0], currents[1]);
    if (microsteps)
      index += SimmyPulseTrainTickDelta(&pulse, 20.0*microsteps/PI, hz, 0);
  }
  return sqrt(sum/(2*hz));
}

static void velocity_ripple(void) {
  double continuous = velocity_error(10000, 0, 0, 0);
  double m16 = velocity_error(10000, 16, 0, 0);
  double m64 = velocity_error(10000, 64, 0, 0);
  double detent = velocity_error(10000, 0, 1, 0);
  double errors = velocity_error(10000, 0, 0, 1);
  double combined = velocity_error(10000, 16, 1, 1);
  assert(continuous < .001);
  assert(m16 > .1 && m16 < .3);
  assert(m64 < m16/2 && m64 > continuous);
  assert(detent > .05 && detent < .15);
  assert(errors > .02);
  assert(combined > m16);
  /* Halve outer dt: pulse timing and held-current approximation converge,
   * as well as the mechanical substep tests above. Absolute RMS fraction. */
  near(combined, velocity_error(20000, 16, 1, 1), .005);
}

int main(void) {
  torque_and_currents();
  detent_and_steps();
  equilibrium_and_friction();
  integration();
  velocity_ripple();
  puts("Stepper tests passed.");
  return 0;
}
