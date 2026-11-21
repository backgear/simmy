/* Independent closed-loop trajectories in the exact reduction overlap:
 * sinusoidal PM, symmetric linear windings, ideal imposed terminal volts.
 * NOT a comparison of six-step/trapezoidal PWM with averaged sine FOC. */
#include "bldc_experiment.h"
#include <stdio.h>
#include <time.h>

static int benchmark(void) {
  double checksum[3];
  puts("model,electrical_steps,cpu_seconds,checksum");
  for (int model=0;model<3;model++) {
    SimmyPmParams p={.polePairs=4,.resistance=.6,.inductance=.002,.backEmfConstant=.08};
    SimmyPmState s; SimmyPmInit(&s); SimmyPmOutput so={0};
    SimmyBldcParams motor=BldcMotor(model==2 ? SIMMY_DELTA : SIMMY_WYE); motor.waveform=SIMMY_PM_SINE;
    if (model==2) {
      for (int j=0;j<3;j++) { motor.resistance[j]*=3; motor.inductance[j]*=3; motor.backEmfConstant[j]*=sqrt(3); }
      motor.electricalOffset=BLDC_PI/6;
    }
    SimmyBldcState detailed; SimmyBldcInit(&detailed); SimmyBldcOutput o={0};
    clock_t start=clock();
    for (int tick=0;tick<500000;tick++) {
      double angle=tick*.00001*20, v[2]={-10*sin(4*angle),10*cos(4*angle)};
      if (!model) { if (!SimmyPmStep(&s,&p,v,angle,20,.00001,&so)) return 1; }
      else {
        double abc[3]={v[0],-.5*v[0]+sqrt(3)/2*v[1],-.5*v[0]-sqrt(3)/2*v[1]};
        if (!SimmyBldcVoltageStep(&detailed,&motor,abc,angle,20,.00001,&o)) return 1;
      }
    }
    double seconds=(double)(clock()-start)/CLOCKS_PER_SEC;
    checksum[model]=!model ? s.windingHeatJ+so.torque : SimmyBldcHeatJ(&detailed.energy)+o.torque;
    printf("%s,500000,%.6f,%.12f\n",model==0 ? "reduced" : model==1 ? "wye" : "equivalent_delta",seconds,checksum[model]);
  }
  return fabs(checksum[0]-checksum[1])<1e-7 && fabs(checksum[0]-checksum[2])<1e-7 ? 0 : 1;
}

int main(int argc,char **argv) {
  if (argc==2 && !strcmp(argv[1],"--benchmark")) return benchmark();
  if (argc!=1) { fprintf(stderr,"Usage: %s [--benchmark]\n",argv[0]); return 1; }
  BldcTrial detailed[2]={BldcTrialInit(0),BldcTrialInit(BLDC_VARIANTS)};
  SimmyPmParams p={.polePairs=4,.resistance=.6,.inductance=.002,.backEmfConstant=.08};
  SimmyPmState simple; SimmyPmInit(&simple); SimmyPmOutput so={0};
  SimmyChainState mechanics; SimmyChainInit(&mechanics,&detailed[0].chain);
  mechanics.bodies[0].position=37; /* same initial condition, no ongoing state copying */
  for (int c=0;c<2;c++) {
    detailed[c].motor.waveform=SIMMY_PM_SINE;
    if (c) {
      for (int j=0;j<3;j++) {
        detailed[c].motor.resistance[j]*=3; detailed[c].motor.inductance[j]*=3;
        detailed[c].motor.backEmfConstant[j]*=sqrt(3);
      }
      detailed[c].motor.electricalOffset=BLDC_PI/6;
    }
  }
  puts("time_s,simple_speed_rad_s,wye_speed_rad_s,delta_speed_rad_s,simple_torque_nm,"
       "wye_torque_nm,delta_torque_nm,simple_heat_j,wye_heat_j,delta_heat_j,max_current_error_a");
  double currentError=0;
  for (int tick=0;tick<100000;tick++) {
    double dt=.00001,time=tick*dt,brake=time>=.4 && time<.8 ? .1 : 0;
    double qVoltage=time<.05 ? 0 : 8;
    /* Each model gets its OWN mechanical feedback and same voltage policy.
     * No speed/current/state copied between models, nor truth-voltage replay. */
    double angle=mechanics.bodies[0].position*.001, speed=mechanics.bodies[0].velocity;
    double v[2]={-qVoltage*sin(4*angle),qVoltage*cos(4*angle)};
    if (!SimmyPmStep(&simple,&p,v,angle,speed,dt,&so)) return 1;
    SimmyChainParams chain=detailed[0].chain;
    chain.bodies[0].coulombLevel=chain.bodies[0].stictionLimit=brake;
    SimmyChainStepDt(&mechanics,&chain,so.torque,dt/.001);
    for (int c=0;c<2;c++) {
      BldcTrial *t=&detailed[c];
      angle=BldcAngle(t); speed=BldcSpeed(t);
      double ab[2]={-qVoltage*sin(4*angle),qVoltage*cos(4*angle)};
      double abc[3]={ab[0],-.5*ab[0]+sqrt(3)/2*ab[1],-.5*ab[0]-sqrt(3)/2*ab[1]};
      if (!SimmyBldcVoltageStep(&t->electrical,&t->motor,abc,angle,speed,dt,&t->output)) return 1;
      t->chain.bodies[0].coulombLevel=t->chain.bodies[0].stictionLimit=brake;
      SimmyChainStepDt(&t->mechanics,&t->chain,t->output.torque,dt/.001);
      currentError=fmax(currentError,fabs(t->output.terminalCurrent[0]-simple.current[0]));
      currentError=fmax(currentError,fabs((t->output.terminalCurrent[1]-t->output.terminalCurrent[2])/sqrt(3)-simple.current[1]));
    }
    if ((tick+1)%100==0) printf("%.6f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12g\n",
        (tick+1)*dt,mechanics.bodies[0].velocity,BldcSpeed(&detailed[0]),BldcSpeed(&detailed[1]),so.torque,
        detailed[0].output.torque,detailed[1].output.torque,simple.windingHeatJ,
        SimmyBldcHeatJ(&detailed[0].electrical.energy),SimmyBldcHeatJ(&detailed[1].electrical.energy),currentError);
  }
  fprintf(stderr,"Independent coupled PM reduction: max alpha/beta current discrepancy %.3g A\n",currentError);
  return currentError<1e-7 ? 0 : 1;
}
