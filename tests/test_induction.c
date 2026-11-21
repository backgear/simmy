#include "simmy.h"
#include "../examples/induction_experiment.h"
#include <assert.h>
#include <complex.h>
#include <stdio.h>
#include <string.h>

static void near(double a, double b, double tolerance) { assert(fabs(a-b) < tolerance); }

static void parameters_and_zero(void) {
  SimmyInductionParams p = InductionMotor();
  assert(SimmyInductionParamsValid(&p));
  p.statorLeakageInductance = 0;
  assert(!SimmyInductionParamsValid(&p));
  p = InductionMotor(); p.rotorResistance = NAN;
  assert(!SimmyInductionParamsValid(&p));
  p = InductionMotor(); p.polePairs = 0;
  assert(!SimmyInductionParamsValid(&p));
  p = InductionMotor();
  SimmyInductionState s;
  SimmyInductionInit(&s);
  for (int i = 0; i < 1000; i++)
    assert(SimmyInductionStep(&s, &p, 0, 0, 100, .000025) == 0);
  SimmyInductionOutput o;
  SimmyInductionEvaluate(&s, &p, &o);
  assert(o.magneticEnergy == 0 && o.torque == 0 && o.statorCurrent[0] == 0);
}

static void flux_and_energy(void) {
  SimmyInductionParams p = InductionMotor();
  double is[2] = {2, -1}, ir[2] = {-.4, .7};
  SimmyInductionState s;
  for (int axis = 0; axis < 2; axis++) {
    s.statorFlux[axis] = (.18+.01)*is[axis] + .18*ir[axis];
    s.rotorFlux[axis] = .18*is[axis] + (.18+.01)*ir[axis];
  }
  SimmyInductionOutput o;
  SimmyInductionEvaluate(&s, &p, &o);
  for (int axis = 0; axis < 2; axis++) {
    near(o.statorCurrent[axis], is[axis], 1e-14);
    near(o.rotorCurrent[axis], ir[axis], 1e-14);
  }
  assert(o.magneticEnergy > 0);
  /* Energy balance independent of the electrical ODE implementation:
   * dW/dt = 3/2 v.i - stator copper - rotor copper - torque*omega. */
  double input = 1.5*(40*is[0]-10*is[1]);
  double loss = 1.5*(3*(is[0]*is[0]+is[1]*is[1])+2*(ir[0]*ir[0]+ir[1]*ir[1]));
  double old = o.magneticEnergy, dt = 1e-9;
  SimmyInductionStep(&s, &p, 40, -10, 90, dt);
  SimmyInductionOutput next;
  SimmyInductionEvaluate(&s, &p, &next);
  near((next.magneticEnergy-old)/dt, input-loss-o.torque*90, .001);

  /* Lossless speed coupling must exchange electrical/mechanical energy
   * with the correct sign and 3/2 and pole-pair factors. */
  p.statorResistance = p.rotorResistance = 0;
  SimmyInductionEvaluate(&s, &p, &o);
  double torque = SimmyInductionStep(&s, &p, 0, 0, 90, 1e-5);
  SimmyInductionEvaluate(&s, &p, &next);
  near(next.magneticEnergy-o.magneticEnergy + torque*90*1e-5, 0, 1e-10);

  p = InductionMotor();
  for (int i = 0; i < 1000; i++) {
    SimmyInductionEvaluate(&s, &p, &o);
    SimmyInductionStep(&s, &p, 0, 0, 0, .000025);
    SimmyInductionEvaluate(&s, &p, &next);
    assert(next.magneticEnergy <= o.magneticEnergy + 1e-14);
  }
}

/* Independent steady-state per-phase equivalent circuit, driven by peak
 * phasors: stator Rs+jXs in series with (jXm || (Rr/slip+jXlr)). */
static void equivalent_circuit(double slip) {
  SimmyInductionParams p = InductionMotor();
  double w = 2*INDUCTION_PI*50, voltage = 230*sqrt(2.0/3);
  double complex zr = p.rotorResistance/slip + I*w*p.rotorLeakageInductance;
  double complex zm = I*w*p.magnetizingInductance;
  double complex zs = p.statorResistance + I*w*p.statorLeakageInductance;
  double complex current = voltage/(zs+zm*zr/(zm+zr));
  double complex rotorCurrent = (voltage-current*zs)/zr;
  double expectedTorque = 1.5*p.polePairs/w*(p.rotorResistance/slip)*pow(cabs(rotorCurrent), 2);
  SimmyInductionState s;
  SimmyInductionInit(&s);
  double dt = .000025, speed = w/p.polePairs*(1-slip);
  for (int i = 0; i < 120000; i++) {
    double phase = w*(i+.5)*dt;
    SimmyInductionStep(&s, &p, voltage*cos(phase), voltage*sin(phase), speed, dt);
  }
  SimmyInductionOutput o;
  SimmyInductionEvaluate(&s, &p, &o);
  near(hypot(o.statorCurrent[0], o.statorCurrent[1])/cabs(current), 1, 5e-5);
  near(o.torque/expectedTorque, 1, 5e-5);
  printf("Equivalent circuit slip %+.2f: torque %.6f Nm (reference %.6f)\n",
         slip, o.torque, expectedTorque);
}

static void direction_and_synchronous_speed(void) {
  SimmyInductionParams p = InductionMotor();
  SimmyInductionState forward, reverse, synchronous;
  SimmyInductionInit(&forward); SimmyInductionInit(&reverse); SimmyInductionInit(&synchronous);
  double dt = .000025, w = 2*INDUCTION_PI*50;
  for (int i = 0; i < 120000; i++) {
    double phase = w*(i+.5)*dt, va = 180*cos(phase), vb = 180*sin(phase);
    SimmyInductionStep(&forward, &p, va, vb, 100, dt);
    SimmyInductionStep(&reverse, &p, va, -vb, -100, dt);
    SimmyInductionStep(&synchronous, &p, va, vb, w/p.polePairs, dt);
  }
  SimmyInductionOutput a, b, sync;
  SimmyInductionEvaluate(&forward, &p, &a); SimmyInductionEvaluate(&reverse, &p, &b);
  SimmyInductionEvaluate(&synchronous, &p, &sync);
  near(a.torque, -b.torque, 1e-12);
  assert(a.torque > 0);
  /* Small held-voltage discretization residual at this integration dt. */
  near(sync.torque, 0, 5e-5); /* no rotor induction at zero slip */
  near(hypot(sync.rotorCurrent[0], sync.rotorCurrent[1]), 0, 2e-4);
}

static void drive_and_chain_duration(void) {
  SimmyVfdParams p = InductionTrialInit(10, 25).driveParams;
  SimmyVfdState s;
  SimmyVfdInit(&s);
  SimmyVfdCommandStep(&s, &p, 50, .001);
  near(s.frequencyHz, .025, 1e-14);
  double voltage[2];
  for (int i = 0; i < 40; i++) SimmyVfdVoltageStep(&s, .000025, voltage);
  near(s.frequencyHz, .025, 1e-14); /* integration does NOT advance ramp */
  p.rampHzPerSecond = 0;
  SimmyVfdCommandStep(&s, &p, 5, .001);
  near(s.phasePeakV, 10+(p.basePhasePeakV-10)*.1, 1e-12);
  SimmyVfdCommandStep(&s, &p, -100, .001);
  near(s.phasePeakV, p.maxPhasePeakV, 1e-12);
  double oldPhase = s.phaseRadians;
  SimmyVfdVoltageStep(&s, .001, voltage);
  near(remainder(s.phaseRadians-oldPhase, 2*INDUCTION_PI), -2*INDUCTION_PI*.1, 1e-12);
  near(hypot(voltage[0], voltage[1]), s.phasePeakV, 1e-12);
  SimmyVfdCommandStep(&s, &p, 0, .001);
  assert(s.phasePeakV == 0);

  for (int bodies = 1; bodies <= 2; bodies++) {
    SimmyChainParams chain = {.bodyCount=bodies, .subSteps=10};
    chain.bodies[0].inertia = chain.bodies[1].inertia = 2;
    chain.couplings[0].springK = .1;
    SimmyChainState a, b;
    SimmyChainInit(&a, &chain); b = a;
    for (int i = 0; i < 10; i++) {
      SimmyChainStep(&a, &chain, .3);
      SimmyChainStepDt(&b, &chain, .3, 1);
    }
    assert(memcmp(&a, &b, sizeof(a)) == 0);
  }
  SimmyChainParams chain = {.bodyCount=1};
  chain.bodies[0].inertia = 2;
  SimmyChainState body;
  SimmyChainInit(&body, &chain);
  SimmyChainStepDt(&body, &chain, 4, .25);
  near(body.bodies[0].velocity, .5, 1e-15);
  near(body.bodies[0].position, .125, 1e-15);
}

static void coupled_loads(void) {
  InductionTrial t = InductionTrialInit(0, 25);
  for (int i = 0; i < 3000; i++) InductionTrialTick(&t, 50, 40);
  double synchronous = 2*INDUCTION_PI*50/2;
  double unloaded = InductionSpeed(&t);
  assert(unloaded < synchronous && unloaded > .99*synchronous);
  t.chain.bodies[0].coulombLevel = t.chain.bodies[0].stictionLimit = 6;
  for (int i = 0; i < 2000; i++) InductionTrialTick(&t, 50, 40);
  double loaded = InductionSpeed(&t);
  assert(loaded < unloaded-3 && loaded > .9*synchronous);
  t.chain.bodies[0].coulombLevel = t.chain.bodies[0].stictionLimit = 20;
  for (int i = 0; i < 2000; i++) InductionTrialTick(&t, 50, 40);
  assert(InductionSpeed(&t) == 0); /* passive brake stalls, cannot back-drive */
  t.chain.bodies[0].coulombLevel = t.chain.bodies[0].stictionLimit = 6;
  for (int i = 0; i < 2000; i++) InductionTrialTick(&t, 50, 40);
  near(InductionSpeed(&t), loaded, .01);

  InductionTrial coarse = InductionTrialInit(0, 25), fine = coarse;
  /* Fixed 1 kHz command clock and original chain units at BOTH resolutions. */
  for (int i = 0; i < 700; i++) {
    InductionTrialTick(&coarse, 50, 20);
    InductionTrialTick(&fine, 50, 40);
  }
  near(coarse.drive.frequencyHz, fine.drive.frequencyHz, 1e-14);
  near(InductionSpeed(&coarse), InductionSpeed(&fine), .02);

  InductionTrial unboosted = InductionTrialInit(0, 10), boosted = InductionTrialInit(10, 10);
  for (int i = 0; i < 3000; i++) {
    if (i == 1000) {
      unboosted.chain.bodies[0].coulombLevel = unboosted.chain.bodies[0].stictionLimit = 3;
      boosted.chain.bodies[0].coulombLevel = boosted.chain.bodies[0].stictionLimit = 3;
    }
    InductionTrialTick(&unboosted, 5, 40);
    InductionTrialTick(&boosted, 5, 40);
  }
  assert(InductionSpeed(&unboosted) == 0);
  assert(InductionSpeed(&boosted) > 10);
  printf("Load test: synchronous %.3f, unloaded %.3f, loaded %.3f rad/s; overload stalls\n",
         synchronous, unloaded, loaded);
}

int main(void) {
  parameters_and_zero(); flux_and_energy();
  equivalent_circuit(1); equivalent_circuit(.04); equivalent_circuit(-.04);
  direction_and_synchronous_speed(); drive_and_chain_duration(); coupled_loads();
  puts("Induction/VFD tests passed.");
  return 0;
}
