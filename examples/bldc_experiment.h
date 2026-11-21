#ifndef BLDC_EXPERIMENT_H
#define BLDC_EXPERIMENT_H
#include "simmy.h"
#include <math.h>
#include <string.h>

#define BLDC_PI 3.14159265358979323846
#define BLDC_VARIANTS 8
#define BLDC_CASES (2*BLDC_VARIANTS)
#define BLDC_PWM_DT .000005 /* fixed 5 us digital PWM clock, NOT solver dt */
#define BLDC_PWM_PER_MS 200
static const char *const BldcCaseNames[BLDC_CASES]={
  "wye_healthy","wye_advance30","wye_wrong_sector","wye_high_r","wye_high_ron","wye_open_high","wye_open_winding","wye_reverse_sector",
  "delta_healthy","delta_advance30","delta_wrong_sector","delta_high_r","delta_high_ron","delta_open_high","delta_open_winding","delta_reverse_sector"
};
typedef struct {
  SimmyBldcParams motor;
  SimmyBridgeParams bridge;
  SimmyBldcState electrical;
  SimmyBldcOutput output;
  SimmyChainParams chain;
  SimmyChainState mechanics;
  SimmyMotorMount mount;
  double speedIntegral, duty, advance, loadWorkJ, errorSquareIntegral, peakCurrent;
  int sectorOffset, sector[3], gate[3];
} BldcTrial;
static inline SimmyBldcParams BldcMotor(int connection) {
  SimmyBldcParams p={.polePairs=4,.connection=connection,.waveform=SIMMY_PM_TRAPEZOID};
  for (int j=0;j<3;j++) { p.resistance[j]=.6; p.inductance[j]=.002; p.backEmfConstant[j]=.08; }
  return p;
}
static inline BldcTrial BldcTrialInit(int caseId) {
  BldcTrial t={0}; t.motor=BldcMotor(caseId/BLDC_VARIANTS);
  t.bridge.busVoltage=24; t.bridge.diodeDrop=.6;
  for (int j=0;j<6;j++) t.bridge.onResistance[j]=.04;
  switch (caseId%BLDC_VARIANTS) {
    case 1: t.advance=BLDC_PI/6; break;
    case 2: t.sectorOffset=1; break;
    case 3: t.motor.resistance[0]*=5; break;
    case 4: t.bridge.onResistance[1]=1.5; break;
    case 5: t.bridge.openSwitch[1]=1; break;
    case 6: t.motor.openWinding[0]=1; break;
    case 7: t.sectorOffset=2; break;
    default: break;
  }
  t.mount=(SimmyMotorMount){.radiansPerCount=.001,.forcePerNm=1,.tickSeconds=.001};
  t.chain.bodyCount=1; t.chain.subSteps=1;
  t.chain.bodies[0].inertia=.0008/(1000*.001*.001);
  t.chain.bodies[0].friction=.0005/(1000*.001);
  SimmyChainInit(&t.mechanics,&t.chain);
  /* Start away from exact sector/zero-torque symmetry boundaries. The
   * same physical angle is used in every trial, not retuned per fault. */
  t.mechanics.bodies[0].position=37; /* .037 mechanical rad */
  SimmyBldcInit(&t.electrical);
  return t;
}
static inline double BldcSpeed(const BldcTrial *t) {
  return t->mechanics.bodies[0].velocity*t->mount.radiansPerCount/t->mount.tickSeconds;
}
static inline double BldcAngle(const BldcTrial *t) {
  return t->mechanics.bodies[0].position*t->mount.radiansPerCount;
}
static inline double BldcReference(double time) { return fmin(60,fmax(0,(time-.1)*200)); }
static inline double BldcBrake(double time) { return time>=.5 && time<1.5 ? .12 : 0; }
/* 1 kHz example speed PI, ideal encoder measurement. No plant-parameter
 * adaptation or current clipping. Feedforward is an explicit nominal
 * assumption, not the damaged motor/device parameters. */
static inline void BldcSpeedTick(BldcTrial *t,double reference,double brake) {
  double error=reference-BldcSpeed(t);
  double ff=(t->motor.connection==SIMMY_WYE ? 2 : 1.2)*.08*reference/24;
  double raw=ff+.006*error+t->speedIntegral;
  t->duty=fmax(0,fmin(.95,raw));
  t->speedIntegral+=.001*(.08*error+40*(t->duty-raw));
  t->chain.bodies[0].coulombLevel=t->chain.bodies[0].stictionLimit=brake;
}
/* 20 kHz six-step selection, 5 kHz high-side PWM with 40 timing quanta.
 * Low selected leg stays ON during PWM off time (except at zero duty).
 * Splitting a PWM quantum refines ONLY the motor/mechanical solver. */
static inline int BldcPwmTick(BldcTrial *t,int pwmTick,int steps,double reference,double brake) {
  if (pwmTick%10==0) {
    double nominalAngle=4*BldcAngle(t)-(t->motor.connection==SIMMY_DELTA ? BLDC_PI/6 : 0);
    SimmyBldcSixStep(nominalAngle,t->advance,t->sectorOffset,t->sector);
  }
  int on=((pwmTick%40)+.5)/40 < t->duty;
  for (int j=0;j<3;j++) t->gate[j]=t->duty<=0 ? 0 : (t->sector[j]>0 ? on : t->sector[j]);
  double dt=BLDC_PWM_DT/steps;
  for (int sub=0;sub<steps;sub++) {
    double speed=BldcSpeed(t), angle=BldcAngle(t);
    if (!SimmyBldcBridgeStep(&t->electrical,&t->motor,&t->bridge,t->gate,angle,speed,dt,&t->output)) return 0;
    for (int j=0;j<3;j++) t->peakCurrent=fmax(t->peakCurrent,fabs(t->electrical.current[j]));
    t->loadWorkJ+=dt*brake*fabs(speed);
    t->errorSquareIntegral+=dt*(reference-speed)*(reference-speed);
    SimmyChainStepDt(&t->mechanics,&t->chain,t->output.torque*t->mount.forcePerNm,dt/t->mount.tickSeconds);
  }
  return 1;
}
#endif
