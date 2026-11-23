/* Stateful adapter for frozen command-v2, for feedback-driven native runs.
 * Same equations/0.5ms step as axis_command_model_v2.c. Empirical command model,
 * not measured current. Original frozen batch kernel remains unchanged.
 */
#ifndef AXIS_COMMAND_STEP_H
#define AXIS_COMMAND_STEP_H
#include <math.h>
typedef struct { double x,v,z; } AxisCommandState;
static inline void axis_command_step(AxisCommandState *s,double raw,double dt,const double *p) {
    double x=s->x,v=s->v,z=s->z;
    double target=copysign(fmax(fabs(raw)-p[7],0),raw);
    double f=-expm1(-dt/p[6]);
    double avg=target+(z-target)*p[6]/dt*f;z+=(target-z)*f;
    double force=(avg>=0?p[0]:p[1])*avg;
    int positive=fabs(v)>1e-10?v>0:force>=0;
    double loss=positive?p[2]:p[3],b=positive?p[4]:p[5];
    double a=force-(positive?loss:-loss)-b*v;
    if(fabs(v)<1e-10 && fabs(force)<=(force>=0?p[2]:p[3]))v=0;
    else {
        double next=v+a*dt;
        if(v*next<0) {
            double tzero=-v/a;x+=.5*v*tzero;
            double remaining=dt-tzero;double opposing=force>=0?p[2]:p[3];
            if(fabs(force)>opposing) {
                double anew=force-copysign(opposing,force);v=anew*remaining;x+=.5*v*remaining;
            } else v=0;
        } else {x+=.5*(v+next)*dt;v=next;}
    }
    s->x=x;s->v=v;s->z=z;
}
#endif
