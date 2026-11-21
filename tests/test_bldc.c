#include "simmy.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define PI 3.14159265358979323846
static void near(double a,double b,double tolerance) { assert(fabs(a-b)<tolerance); }
static SimmyBldcParams Motor(int connection) {
  SimmyBldcParams p={.polePairs=4,.connection=connection,.waveform=SIMMY_PM_SINE};
  for (int j=0;j<3;j++) { p.resistance[j]=.6; p.inductance[j]=.002; p.backEmfConstant[j]=.08; }
  return p;
}
static SimmyBridgeParams Bridge(void) {
  SimmyBridgeParams p={.busVoltage=24,.diodeDrop=.6};
  for (int j=0;j<6;j++) p.onResistance[j]=.04;
  return p;
}
static double Magnetic(const SimmyBldcState *s,const SimmyBldcParams *p) {
  double w=0; for (int j=0;j<3;j++) w+=.5*p->inductance[j]*s->current[j]*s->current[j];
  return w;
}
static void Ledger(const SimmyBldcState *s,const SimmyBldcParams *p,double initial) {
  near(s->energy.inputJ-s->energy.mechanicalWorkJ-SimmyBldcHeatJ(&s->energy)
       -Magnetic(s,p)+initial,s->energy.numericalLossJ,2e-8);
}
static void equivalence(void) {
  double worst=0;
  for (int wave=0;wave<2;wave++) for (int connection=0;connection<2;connection++) {
    if (wave && connection==SIMMY_DELTA) continue; /* circulating mode is outside reduction */
    SimmyBldcParams p=Motor(connection); p.waveform=wave;
    if (connection==SIMMY_WYE) p.thirdHarmonic=.17; /* neutral blocks zero-sequence current */
    SimmyPmParams reduced={.polePairs=4,.waveform=wave,.resistance=.6,.inductance=.002,.backEmfConstant=.08};
    if (connection==SIMMY_DELTA) {
      for (int j=0;j<3;j++) { p.resistance[j]*=3; p.inductance[j]*=3; p.backEmfConstant[j]*=sqrt(3); }
      p.electricalOffset=PI/6;
    }
    SimmyBldcState s; SimmyBldcInit(&s);
    SimmyPmState r; SimmyPmInit(&r);
    double angle=0;
    for (int tick=0;tick<20000;tick++) {
      double dt=.00001, t=tick*dt, speed=30*sin(20*t);
      double v[2]={10*sin(113*t),9*cos(83*t)}, abc[3];
      abc[0]=v[0]+3; abc[1]=-.5*v[0]+sqrt(3)/2*v[1]+3; abc[2]=-.5*v[0]-sqrt(3)/2*v[1]+3;
      SimmyBldcOutput o; SimmyPmOutput ro;
      assert(SimmyBldcVoltageStep(&s,&p,abc,angle,speed,dt,&o));
      assert(SimmyPmStep(&r,&reduced,v,angle,speed,dt,&ro));
      angle+=speed*dt;
      near(o.terminalCurrent[0],r.current[0],1e-10);
      near((o.terminalCurrent[1]-o.terminalCurrent[2])/sqrt(3),r.current[1],1e-10);
      near(o.torque,ro.torque,1e-11); near(o.magneticEnergy,ro.magneticEnergy,1e-11);
      near(SimmyBldcHeatJ(&s.energy),r.windingHeatJ,1e-9);
      worst=fmax(worst,fabs(o.torque-ro.torque));
    }
    Ledger(&s,&p,0);
    near(r.inputJ-r.mechanicalWorkJ-r.windingHeatJ-.75*reduced.inductance*
         (r.current[0]*r.current[0]+r.current[1]*r.current[1]),r.numericalLossJ,1e-8);
  }
  printf("PM reduction: sine/trap wye + equivalent sine delta match currents/torque/energy/heat (max torque error %.3g Nm)\n",worst);
}
static void bridge_and_freewheel(void) {
  for (int connection=0;connection<2;connection++) {
    SimmyBldcParams p=Motor(connection); SimmyBridgeParams b=Bridge();
    SimmyBldcState s; SimmyBldcInit(&s); SimmyBldcOutput o;
    int command[3]={1,-1,0}; double dt=.00001;
    for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&s,&p,&b,command,0,0,dt,&o));
    assert(o.terminalCurrent[0]>10); near(o.terminalCurrent[2],0,1e-10);
    near(o.terminalCurrent[0]+o.terminalCurrent[1]+o.terminalCurrent[2],0,1e-10);
    double before=Magnetic(&s,&p), oldCurrent=o.terminalCurrent[0], beforeHeat=SimmyBldcHeatJ(&s.energy);
    command[0]=command[1]=command[2]=0;
    assert(SimmyBldcBridgeStep(&s,&p,&b,command,0,0,dt,&o));
    assert(o.terminalCurrent[0]>oldCurrent*.9); /* not abruptly zeroed */
    assert(o.inputPower<0); /* returned to ideal DC bus through diodes */
    assert(o.diodeLossW[0]>0 && o.diodeLossW[3]>0);
    for (int t=0;t<2000;t++) assert(SimmyBldcBridgeStep(&s,&p,&b,command,0,0,dt,&o));
    assert(Magnetic(&s,&p)<before*.001);
    assert(SimmyBldcHeatJ(&s.energy)>beforeHeat);
    Ledger(&s,&p,0);
    /* OFF is not disconnected: overspeed rectifies through body diodes. */
    assert(SimmyBldcBridgeStep(&s,&p,&b,command,.1,500,dt,&o));
    assert(o.inputPower<0 && o.mechanicalPower<0);
    Ledger(&s,&p,0);
  }
}
static void delta_circulation(void) {
  SimmyBridgeParams b=Bridge(); int off[3]={0};
  double heat[2]={0};
  for (int connection=0;connection<2;connection++) {
    SimmyBldcParams p=Motor(connection); p.waveform=SIMMY_PM_TRAPEZOID;
    SimmyBldcState s; SimmyBldcInit(&s); SimmyBldcOutput o;
    for (int t=0;t<10000;t++) {
      assert(SimmyBldcBridgeStep(&s,&p,&b,off,t*.00001*100,100,.00001,&o));
      for (int j=0;j<3;j++) near(o.terminalCurrent[j],0,1e-10);
    }
    heat[connection]=SimmyBldcHeatJ(&s.energy);
    near(s.energy.inputJ,0,1e-10);
    Ledger(&s,&p,0);
    if (connection==SIMMY_DELTA) assert(fabs(s.current[0])>.1);
  }
  printf("Open-terminal trapezoidal PM: floating wye heat %.5g J; closed delta %.5g J despite zero line currents\n",heat[0],heat[1]);
  assert(heat[SIMMY_WYE]<1e-20 && heat[SIMMY_DELTA]>.01);
}
static void faults_and_validation(void) {
  SimmyBldcParams p=Motor(SIMMY_WYE); SimmyBridgeParams b=Bridge();
  SimmyBldcState healthy,bad; SimmyBldcInit(&healthy); SimmyBldcInit(&bad);
  SimmyBldcOutput h,o; int drive[3]={1,-1,0};
  b.openSwitch[1]=1;
  for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&bad,&p,&b,drive,0,0,.00001,&o));
  near(o.terminalCurrent[0],0,1e-10); /* high FET can't source current */
  b.openSwitch[1]=0;
  for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&healthy,&p,&b,drive,0,0,.00001,&h));
  assert(h.terminalCurrent[0]>10);
  near(h.torque,-.08*sqrt(3)/2*h.terminalCurrent[0],1e-12);
  assert(h.mechanicalPower==0 && h.inputPower>0); /* stall torque, copper heating, no shaft work */
  /* A failed HIGH FET is not an open winding: low channel still works. */
  b.openSwitch[1]=1; SimmyBldcInit(&bad); drive[0]=-1; drive[1]=1;
  for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&bad,&p,&b,drive,0,0,.00001,&o));
  assert(o.terminalCurrent[0]<-10); Ledger(&bad,&p,0);
  /* Increased motor R vs increased transistor R have separate heat ledgers. */
  b=Bridge(); drive[0]=1; drive[1]=-1; SimmyBldcInit(&bad); p.resistance[0]*=5;
  for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&bad,&p,&b,drive,0,0,.00001,&o));
  assert(o.terminalCurrent[0]<h.terminalCurrent[0]);
  assert(bad.energy.windingHeatJ[0]>bad.energy.windingHeatJ[1]*4.9);
  Ledger(&bad,&p,0);
  p=Motor(SIMMY_WYE); b.onResistance[1]=3; SimmyBldcInit(&bad);
  for (int t=0;t<1000;t++) assert(SimmyBldcBridgeStep(&bad,&p,&b,drive,0,0,.00001,&o));
  assert(bad.energy.switchHeatJ[1]>bad.energy.switchHeatJ[2]*10); Ledger(&bad,&p,0);
  /* Opening an energized winding is deliberately unsupported, never erased. */
  SimmyBldcState saved=healthy; SimmyBldcOutput sentinel; memset(&sentinel,0x42,sizeof(sentinel)); o=sentinel;
  p.openWinding[0]=1;
  assert(!SimmyBldcBridgeStep(&healthy,&p,&b,drive,0,0,.00001,&o));
  assert(!memcmp(&healthy,&saved,sizeof(saved)) && !memcmp(&o,&sentinel,sizeof(o)));
  for (int c=0;c<2;c++) for (int mask=1;mask<8;mask++) {
    p=Motor(c); SimmyBldcInit(&bad); b=Bridge();
    for (int j=0;j<3;j++) p.openWinding[j]=(mask>>j)&1;
    for (int t=0;t<100;t++) {
      assert(SimmyBldcBridgeStep(&bad,&p,&b,drive,30*.00001*t,30,.00001,&o));
      for (int j=0;j<3;j++) if (p.openWinding[j]) assert(bad.current[j]==0);
    }
    Ledger(&bad,&p,0);
  }
  p=Motor(0); p.inductance[1]=0; assert(!SimmyBldcParamsValid(&p));
  p=Motor(0); p.resistance[1]=-1; assert(!SimmyBldcParamsValid(&p));
  b=Bridge(); b.busVoltage=NAN; assert(!SimmyBridgeParamsValid(&b));
  SimmyPmState r; SimmyPmInit(&r); SimmyPmParams rp={.polePairs=4,.resistance=.6,.inductance=.002,.backEmfConstant=.08};
  SimmyPmOutput ro; double v[2]={NAN,0};
  assert(!SimmyPmStep(&r,&rp,v,0,0,.001,&ro));
  /* Unrepresentable energy must fail atomically, not commit infinities. */
  p=Motor(SIMMY_DELTA); SimmyBldcInit(&bad);
  for (int j=0;j<3;j++) bad.current[j]=1e200;
  saved=bad; o=sentinel; double zero[3]={0};
  assert(!SimmyBldcVoltageStep(&bad,&p,zero,0,0,.001,&o));
  assert(!memcmp(&bad,&saved,sizeof(saved)) && !memcmp(&o,&sentinel,sizeof(o)));
}
static double rl_error(double dt,double *numerical) {
  SimmyBldcParams p=Motor(SIMMY_WYE);
  for (int j=0;j<3;j++) p.backEmfConstant[j]=0;
  SimmyBldcState s; SimmyBldcInit(&s); SimmyBldcOutput o;
  double v[3]={12,-12,0}; int n=(int)round(.01/dt);
  for (int i=0;i<n;i++) assert(SimmyBldcVoltageStep(&s,&p,v,0,0,dt,&o));
  double exact=12/.6*(1-exp(-.6/.002*.01));
  Ledger(&s,&p,0); *numerical=s.energy.numericalLossJ;
  return fabs(s.current[0]-exact);
}
static void convergence_and_commutation(void) {
  double a,b; double e1=rl_error(.0001,&a),e2=rl_error(.00005,&b);
  assert(e2<e1*.55 && b<a*.55);
  for (int sector=0;sector<6;sector++) {
    int cmd[3],shift[3]; double angle=.123+sector*PI/3;
    SimmyBldcSixStep(angle,0,0,cmd); SimmyBldcSixStep(angle+PI/3,0,0,shift);
    int plus=0,minus=0,zero=0;
    for (int j=0;j<3;j++) { plus+=cmd[j]==1; minus+=cmd[j]==-1; zero+=cmd[j]==0; }
    assert(plus==1 && minus==1 && zero==1);
    SimmyBldcSixStep(angle,0,1,cmd); assert(!memcmp(cmd,shift,sizeof(cmd)));
  }
}
static unsigned rng=12345;
static double random_unit(void) {
  rng=1664525*rng+1013904223; return (rng>>8)/16777216.0;
}
static void circuit_property_sweep(void) {
  for (int trial=0;trial<3000;trial++) {
    SimmyBldcParams p=Motor(trial%2); p.waveform=trial%3 ? SIMMY_PM_SINE : SIMMY_PM_TRAPEZOID;
    SimmyBridgeParams b=Bridge(); b.busVoltage=3+45*random_unit(); b.diodeDrop=trial%5 ? .7 : 0;
    SimmyBldcState s; SimmyBldcInit(&s);
    s.current[0]=10*(random_unit()-.5); s.current[1]=10*(random_unit()-.5);
    s.current[2]=p.connection==SIMMY_WYE ? -s.current[0]-s.current[1] : 10*(random_unit()-.5);
    for (int j=0;j<3;j++) {
      p.resistance[j]=.1+3*random_unit(); p.inductance[j]=.001+.004*random_unit();
      p.backEmfConstant[j]=.02+.13*random_unit();
    }
    int gate[3];
    for (int j=0;j<3;j++) gate[j]=(int)(3*random_unit())-1;
    for (int j=0;j<6;j++) { b.onResistance[j]=trial%7 ? .01+2*random_unit() : 0; b.openSwitch[j]=(trial+j)%11==0; }
    double initial=Magnetic(&s,&p); SimmyBldcOutput o;
    assert(SimmyBldcBridgeStep(&s,&p,&b,gate,10*random_unit(),600*(random_unit()-.5),.00001,&o));
    Ledger(&s,&p,initial);
    near(o.terminalCurrent[0]+o.terminalCurrent[1]+o.terminalCurrent[2],0,1e-9);
    for (int j=0;j<3;j++) {
      assert(o.terminalVoltage[j]>=-b.diodeDrop-1e-8 && o.terminalVoltage[j]<=b.busVoltage+b.diodeDrop+1e-8);
      assert(o.windingLossW[j]>=0);
    }
    for (int j=0;j<6;j++) assert(o.switchLossW[j]>=0 && o.diodeLossW[j]>=0);
  }
}
int main(void) {
  equivalence(); bridge_and_freewheel(); delta_circulation(); faults_and_validation(); convergence_and_commutation();
  circuit_property_sweep();
  puts("BLDC/PM circuit, heat and reduction tests passed."); return 0;
}
