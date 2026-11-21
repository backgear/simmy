/* Electrical dynamometer fixture: externally driven shaft at 100 rad/s.
 * All FETs OFF; at this speed body diodes do not conduct. No external line
 * current, but trapezoidal delta windings retain an internal loop. */
#include "bldc_experiment.h"
#include <stdio.h>
int main(void) {
  SimmyBldcParams p[2]={BldcMotor(SIMMY_WYE),BldcMotor(SIMMY_DELTA)};
  SimmyBldcState s[2]; SimmyBldcInit(&s[0]); SimmyBldcInit(&s[1]);
  SimmyBridgeParams b={.busVoltage=24,.diodeDrop=.6};
  SimmyBldcOutput o[2]; int off[3]={0};
  puts("time_s,wye_winding_a_a,delta_winding_ab_a,delta_terminal_a_a,wye_heat_j,delta_heat_j,delta_mechanical_power_w");
  for (int tick=0;tick<10000;tick++) {
    for (int c=0;c<2;c++) if (!SimmyBldcBridgeStep(&s[c],&p[c],&b,off,tick*.00001*100,100,.00001,&o[c])) return 1;
    if ((tick+1)%10==0) printf("%.6f,%.12f,%.12f,%.12f,%.12f,%.12f,%.12f\n",(tick+1)*.00001,
      s[0].current[0],s[1].current[0],o[1].terminalCurrent[0],SimmyBldcHeatJ(&s[0].energy),
      SimmyBldcHeatJ(&s[1].energy),o[1].mechanicalPower);
  }
  return 0;
}
