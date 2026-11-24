/* Experimental v3c: v3 elastic state plus a contact-strength state in [0,1].
 * p[0..11]: speed-yield v3 parameters. p[12]: healing time (s),
 * p[13]: sliding renewal distance (rad). At rest strength recovers toward 1;
 * sliding reduces it. Strength changes only yield bounds, never creates
 * elastic force. All captures assume initially healed contacts and zero load.
 * Effective hypothesis, not measured contact physics/current/torque.
 */
#define axis_model axis_model_speed_yield
#include "axis_command_model_v3.c"
#undef axis_model

void axis_model(const double *u,const int *lengths,int cases,int stride,
                double dt,const double *p,double *xout,double *vout) {
    for(int c=0;c<cases;c++) {
        double x=0,v=0,z=0,h=0,age=1,local[12];int base=c*stride;
        for(int j=0;j<12;j++)local[j]=p[j];
        xout[base]=0;vout[base]=0;
        for(int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1];
            double target=copysign(fmax(fabs(raw)-p[7],0),raw);
            double f=-expm1(-dt/p[6]);
            double avg=target+(z-target)*p[6]/dt*f;z+=(target-z)*f;
            double force=(avg>=0?p[0]:p[1])*avg;
            local[8]=p[8]*age;local[9]=p[9]*age;
            double previous=v;
            x+=advance(force,&v,&h,dt,local);
            double speed=.5*(fabs(previous)+fabs(v));
            double rate=1/p[12]+speed/p[13],equilibrium=(1/p[12])/rate;
            age+=(equilibrium-age)*(-expm1(-rate*dt));
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
