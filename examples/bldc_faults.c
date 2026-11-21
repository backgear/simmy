/* Voltage-driven six-step BLDC faults, real PWM and electrical heat ledger.
 * --list enumerates independent trials; run_bldc_sweep.py parallelizes them.
 * 1 ms overview plus a 5 us waveform window, 1.25..1.29 s. */
#include "bldc_experiment.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

static double sum(const double *a,int n) { double r=0; for (int j=0;j<n;j++) r+=a[j]; return r; }
static void row(const BldcTrial *t,int id,int pwm,double ref,double brake) {
  const SimmyBldcEnergy *e=&t->electrical.energy;
  printf("%s,%.9f,%d,%.9f,%.9f,%.9f,%.9f,%.9f",BldcCaseNames[id],(pwm+1)*BLDC_PWM_DT,
         (pwm+1)%BLDC_PWM_PER_MS==0,ref,BldcSpeed(t),brake,t->duty,t->output.torque);
  for (int j=0;j<3;j++) printf(",%.9f",t->electrical.current[j]);
  for (int j=0;j<3;j++) printf(",%.9f",t->output.terminalCurrent[j]);
  for (int j=0;j<3;j++) printf(",%.9f",t->output.terminalVoltage[j]);
  for (int j=0;j<3;j++) printf(",%d",t->gate[j]);
  for (int j=0;j<3;j++) printf(",%.9f",e->windingHeatJ[j]);
  for (int j=0;j<6;j++) printf(",%.9f",e->switchHeatJ[j]);
  printf(",%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",sum(e->diodeHeatJ,6),
         sum(e->switchHeatJ,6),SimmyBldcHeatJ(e),e->inputJ,e->mechanicalWorkJ,t->loadWorkJ,
         e->numericalLossJ,t->output.magneticEnergy,t->errorSquareIntegral,BldcAngle(t),t->peakCurrent);
}
int main(int argc,char **argv) {
  int id=0,steps=4;
  for (int i=1;i<argc;i++) {
    if (!strcmp(argv[i],"--list")) {
      for (int j=0;j<BLDC_CASES;j++) puts(BldcCaseNames[j]);
      return 0;
    } else if (!strcmp(argv[i],"--case") && i+1<argc) {
      id=-1; i++; for (int j=0;j<BLDC_CASES;j++) if (!strcmp(argv[i],BldcCaseNames[j])) id=j;
      if (id<0) { fprintf(stderr,"unknown case: %s\n",argv[i]); return 1; }
    } else if (!strcmp(argv[i],"--steps") && i+1<argc) {
      char *end; errno=0; long value=strtol(argv[++i],&end,10);
      if (errno || end==argv[i] || *end || value<1 || value>1000) { fprintf(stderr,"steps: integer 1..1000\n"); return 1; }
      steps=(int)value;
    } else { fprintf(stderr,"Usage: %s [--list] [--case NAME] [--steps PLANT_STEPS_PER_5US_PWM_QUANTUM]\n",argv[0]); return 1; }
  }
  BldcTrial t=BldcTrialInit(id);
  puts("case,time_s,command_sample,reference_rad_s,speed_rad_s,brake_nm,duty,torque_nm,"
       "iwa_a,iwb_a,iwc_a,ila_a,ilb_a,ilc_a,va_v,vb_v,vc_v,gate_a,gate_b,gate_c,"
       "winding_a_heat_j,winding_b_heat_j,winding_c_heat_j,"
       "fet0_heat_j,fet1_heat_j,fet2_heat_j,fet3_heat_j,fet4_heat_j,fet5_heat_j,"
       "diode_heat_j,switch_heat_j,heat_j,input_j,mechanical_work_j,load_work_j,"
       "numerical_loss_j,magnetic_energy_j,error_square_integral,angle_rad,peak_winding_current_a");
  double ref=0,brake=0;
  for (int pwm=0;pwm<400000;pwm++) {
    double time=pwm*BLDC_PWM_DT;
    if (pwm%BLDC_PWM_PER_MS==0) {
      ref=BldcReference(time); brake=BldcBrake(time); BldcSpeedTick(&t,ref,brake);
    }
    if (!BldcPwmTick(&t,pwm,steps,ref,brake)) {
      fprintf(stderr,"circuit failed: %s at %.9f s\n",BldcCaseNames[id],time); return 1;
    }
    if ((pwm+1)%BLDC_PWM_PER_MS==0 || (pwm>=250000 && pwm<258000)) row(&t,id,pwm,ref,brake);
  }
  fprintf(stderr,"%s: heat %.5f J, bus %.5f J, load %.5f J, peak winding current %.4f A\n",
          BldcCaseNames[id],SimmyBldcHeatJ(&t.electrical.energy),t.electrical.energy.inputJ,t.loadWorkJ,t.peakCurrent);
  return 0;
}
