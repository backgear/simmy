#include "../examples/bldc_experiment.h"
#include <assert.h>
#include <stdio.h>

typedef struct { double heat,work,rms,fet1Heat,phaseAHeat; } Result;
static Result run(int caseId) {
  BldcTrial t=BldcTrialInit(caseId);
  double heat0=0,work0=0,error0=0,device0=0,phase0=0,ref=0,brake=0;
  Result r={0};
  for (int pwm=0;pwm<300000;pwm++) {
    double time=pwm*BLDC_PWM_DT;
    if (pwm%200==0) { ref=BldcReference(time); brake=BldcBrake(time); BldcSpeedTick(&t,ref,brake); }
    assert(BldcPwmTick(&t,pwm,2,ref,brake));
    if (pwm==99999) {
      heat0=SimmyBldcHeatJ(&t.electrical.energy); work0=t.loadWorkJ; error0=t.errorSquareIntegral;
      device0=t.electrical.energy.switchHeatJ[1]; phase0=t.electrical.energy.windingHeatJ[0];
    }
  }
  r.heat=SimmyBldcHeatJ(&t.electrical.energy)-heat0;
  r.work=t.loadWorkJ-work0; r.rms=sqrt(t.errorSquareIntegral-error0);
  r.fet1Heat=t.electrical.energy.switchHeatJ[1]-device0;
  r.phaseAHeat=t.electrical.energy.windingHeatJ[0]-phase0;
  double balance=t.electrical.energy.inputJ-t.electrical.energy.mechanicalWorkJ-
      SimmyBldcHeatJ(&t.electrical.energy)-t.output.magneticEnergy-t.electrical.energy.numericalLossJ;
  assert(fabs(balance)<1e-7);
  return r;
}
int main(void) {
  Result healthy=run(0),highR=run(3),highRon=run(4),openHigh=run(5),openWinding=run(6),reverse=run(7);
  assert(healthy.rms<1 && highR.rms<2 && highRon.rms<2);
  assert(highR.heat>healthy.heat*1.5);
  assert(highRon.fet1Heat>healthy.fet1Heat*3);
  assert(highR.phaseAHeat>healthy.phaseAHeat*1.5);
  assert(openHigh.phaseAHeat>0 && openWinding.phaseAHeat==0);
  assert(fabs(highR.work/healthy.work-1)<.03 && fabs(highRon.work/healthy.work-1)<.03);
  assert(reverse.rms>20); /* reversed/failed tracking is never an efficiency success */
  Result delta=run(BLDC_VARIANTS),deltaReverse=run(BLDC_VARIANTS+7);
  assert(delta.rms<1 && deltaReverse.rms>20);
  printf("BLDC matched loaded task: wye heat %.4f J, high winding R %.4f, high FET R %.4f; reverse-table tracking rejected\n",
         healthy.heat,highR.heat,highRon.heat);
  puts("BLDC coupled motion/fault/heat regressions passed."); return 0;
}
