/* Read-only native replay adapter. Preserve state across recorded runs and
 * integrate known zero-output gaps, including their sub-sample remainder.
 * State: x,v,filtered command,elastic load,contact strength,slip mode.
 * Family 0=v2, 1=speed-yield, 2=slip-yield, 3=age-yield.
 * The original batch kernels remain available for equivalence tests.
 */
#include <stdint.h>
#include "axis_command_step.h"
#define axis_model axis_speed_batch
#define advance speed_advance
#include "axis_command_model_v3.c"
#undef advance
#undef axis_model
#define axis_model axis_slip_batch
#define advance slip_advance
#include "axis_command_model_v3_slip.c"
#undef advance
#undef axis_model

static void step(int family,double raw,double dt,const double *p,double *s) {
    if(family==0) {
        AxisCommandState v2={s[0],s[1],s[2]};axis_command_step(&v2,raw,dt,p);
        s[0]=v2.x;s[1]=v2.v;s[2]=v2.z;return;
    }
    double target=copysign(fmax(fabs(raw)-p[7],0),raw);
    double f=-expm1(-dt/p[6]);
    double avg=target+(s[2]-target)*p[6]/dt*f;s[2]+=(target-s[2])*f;
    double force=(avg>=0?p[0]:p[1])*avg,previous=s[1];
    if(family==2) {
        int slipping=(int)s[5];s[0]+=slip_advance(force,&s[1],&s[3],&slipping,dt,p);s[5]=slipping;
    } else {
        double local[12];for(int j=0;j<12;j++)local[j]=p[j];
        if(family==3){local[8]*=s[4];local[9]*=s[4];}
        s[0]+=speed_advance(force,&s[1],&s[3],dt,local);
        if(family==3) {
            double speed=.5*(fabs(previous)+fabs(s[1]));
            double rate=1/p[12]+speed/p[13],equilibrium=(1/p[12])/rate;
            s[4]+=(equilibrium-s[4])*(-expm1(-rate*dt));
        }
    }
}
int axis_state_run(int family,const double *u,int length,double dt,const double *p,
                   double *state,double *xout,double *vout) {
    if(family<0 || family>3 || length<1 || dt<=0)return 0;
    xout[0]=state[0];vout[0]=state[1];
    for(int i=1;i<length;i++) {
        step(family,u[i-1],dt,p,state);xout[i]=state[0];vout[i]=state[1];
    }
    return 1;
}
int axis_state_gap(int family,int64_t ticks,const double *p,double *state) {
    if(family<0 || family>3 || ticks<0)return 0;
    while(ticks>=500){step(family,0,.0005,p,state);ticks-=500;}
    if(ticks)step(family,0,ticks/1000000.,p,state);
    return 1;
}
