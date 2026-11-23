/* Numerical electrical-solver study, NOT a motor's attainable speed rating.
 * Externally imposed shaft speed; default winding/bridge, fixed 60% PWM.
 * Compare identical digital gate/angle inputs at two integration grids.
 * High RPM at 24 V is primarily GENERATING, not a feasible motoring move.
 * Reuse: run_bldc_sweep.py --binary build/bldc_accuracy --output CSV
 */
#include "bldc_experiment.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>

static const int speeds[]={300,600,1500,3000,6000,9000,12000,15000,18000,24000,30000};
#define POINTS ((int)(sizeof(speeds)/sizeof(speeds[0])))

static int integer(const char *text,int *out) {
  char *end; errno=0; long n=strtol(text,&end,10);
  if (errno || end==text || *end || n<1 || n>10000) return 0;
  *out=(int)n; return 1;
}
int main(int argc,char **argv) {
  int connection=0,rpm=6000,steps=4,referenceSteps=100;
  char name[64]="wye_6000";
  for (int i=1;i<argc;i++) {
    if (!strcmp(argv[i],"--list")) {
      for (int c=0;c<2;c++) for (int n=0;n<POINTS;n++) printf("%s_%d\n",c ? "delta" : "wye",speeds[n]);
      return 0;
    } else if (!strcmp(argv[i],"--case") && i+1<argc) {
      int found=0; i++;
      for (int c=0;c<2;c++) for (int n=0;n<POINTS;n++) {
        char candidate[64]; snprintf(candidate,sizeof(candidate),"%s_%d",c ? "delta" : "wye",speeds[n]);
        if (!strcmp(candidate,argv[i])) { connection=c; rpm=speeds[n]; found=1; }
      }
      if (!found) { fprintf(stderr,"unknown accuracy case\n"); return 1; }
      snprintf(name,sizeof(name),"%s",argv[i]);
    } else if ((!strcmp(argv[i],"--steps") || !strcmp(argv[i],"--reference-steps")) && i+1<argc) {
      int reference=!strcmp(argv[i],"--reference-steps");
      if (!integer(argv[++i],reference ? &referenceSteps : &steps)) { fprintf(stderr,"steps: 1..10000\n"); return 1; }
    } else { fprintf(stderr,"Usage: %s [--list] [--case wye_RPM|delta_RPM] [--steps N] [--reference-steps N]\n",argv[0]); return 1; }
  }
  if (referenceSteps<=steps || referenceSteps%steps) { fprintf(stderr,"reference steps must be a larger multiple of steps\n"); return 1; }
  BldcTrial nominal=BldcTrialInit(connection*BLDC_VARIANTS);
  SimmyBldcState state[2]; SimmyBldcInit(&state[0]); SimmyBldcInit(&state[1]);
  SimmyBldcOutput output[2];
  double omega=rpm*2*BLDC_PI/60, dt=BLDC_PWM_DT/steps, refDt=BLDC_PWM_DT/referenceSteps;
  int ratio=referenceSteps/steps;
  int warm=(int)lround(.05/BLDC_PWM_DT);
  int count=(int)ceil(fmax(.03,180.0/(4*rpm))/BLDC_PWM_DT);
  double heat0[2]={0},work0[2]={0},numeric0[2]={0},currentError=0,currentReference=0;
  int sector[3]={0},gate[3];
  for (int pwm=0;pwm<warm+count;pwm++) {
    if (pwm==warm) for (int m=0;m<2;m++) {
      heat0[m]=SimmyBldcHeatJ(&state[m].energy); work0[m]=state[m].energy.mechanicalWorkJ;
      numeric0[m]=state[m].energy.numericalLossJ;
    }
    double angle=.037+omega*pwm*BLDC_PWM_DT;
    if (pwm%10==0) SimmyBldcSixStep(4*angle-(connection ? BLDC_PI/6 : 0),0,0,sector);
    int on=(pwm%40)<24; /* same fixed 60% PWM and 20 kHz commutation for BOTH grids */
    for (int j=0;j<3;j++) gate[j]=sector[j]>0 ? on : sector[j];
    for (int sub=0;sub<steps;sub++) {
      if (!SimmyBldcBridgeStep(&state[0],&nominal.motor,&nominal.bridge,gate,
          angle+omega*sub*dt,omega,dt,&output[0])) return 1;
      for (int r=0;r<ratio;r++) if (!SimmyBldcBridgeStep(&state[1],&nominal.motor,&nominal.bridge,gate,
          angle+omega*(sub*ratio+r)*refDt,omega,refDt,&output[1])) return 1;
      if (pwm>=warm) for (int j=0;j<3;j++) {
        double d=state[0].current[j]-state[1].current[j];
        currentError+=dt*d*d; currentReference+=dt*state[1].current[j]*state[1].current[j];
      }
    }
  }
  double heat[2],torque[2],numeric[2],time=count*BLDC_PWM_DT;
  for (int m=0;m<2;m++) {
    heat[m]=SimmyBldcHeatJ(&state[m].energy)-heat0[m];
    torque[m]=(state[m].energy.mechanicalWorkJ-work0[m])/(omega*time);
    numeric[m]=(state[m].energy.numericalLossJ-numeric0[m])/time;
  }
  /* No cancellation-denominator floor: expose reference mean torque so a
   * near-zero torque point can be recognized instead of overinterpreted. */
  if (fabs(torque[1])<1e-9 || heat[1]<=0 || currentReference<=0) return 1;
  puts("case,rpm,steps,reference_steps,dt_s,measurement_s,current_nrmse,heat_relative_error,"
       "mean_torque_relative_error,coarse_heat_j,reference_heat_j,coarse_mean_torque_nm,reference_mean_torque_nm,"
       "coarse_numerical_loss_w,reference_numerical_loss_w,pwm_cycles_per_electrical_cycle,commutation_samples_per_sector");
  double fe=4*rpm/60.0;
  printf("%s,%d,%d,%d,%.12g,%.9f,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g\n",
      name,rpm,steps,referenceSteps,dt,time,sqrt(currentError/currentReference),fabs(heat[0]/heat[1]-1),
      fabs(torque[0]/torque[1]-1),heat[0],heat[1],torque[0],torque[1],numeric[0],numeric[1],5000/fe,20000/(6*fe));
  return 0;
}
