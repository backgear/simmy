/* Experimental v2 + effective low-speed friction, NOT promoted.
 * p[0..7]: frozen v2 ordering; p[8]: extra static loss (rad/s^2);
 * p[9]: Stribeck velocity scale (rad/s), strictly positive.
 * Extra loss decays as exp(-(v/p[9])^2). No rotor-angle/history state.
 * p[8] == 0 must reproduce frozen v2; parameters are not calibrated torque.
 */
#include <math.h>
void axis_model(const double *u,const int *lengths,int cases,int stride,
                double dt,const double *p,double *xout,double *vout) {
    for(int c=0;c<cases;c++) {
        double x=0,v=0,z=0;int base=c*stride;
        xout[base]=0;vout[base]=0;
        for(int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1];
            double target=copysign(fmax(fabs(raw)-p[7],0),raw);
            double f=-expm1(-dt/p[6]);
            double avg=target+(z-target)*p[6]/dt*f;z+=(target-z)*f;
            double force=(avg>=0?p[0]:p[1])*avg;
            int positive=fabs(v)>1e-10?v>0:force>=0;
            double loss=(positive?p[2]:p[3])+p[8]*exp(-(v/p[9])*(v/p[9]));
            double b=positive?p[4]:p[5];
            double a=force-(positive?loss:-loss)-b*v;
            if(fabs(v)<1e-10 && fabs(force)<=(force>=0?p[2]:p[3])+p[8]) v=0;
            else {
                double next=v+a*dt;
                if(v*next<0) {
                    double tzero=-v/a;x+=.5*v*tzero;
                    double remaining=dt-tzero;
                    double opposing=(force>=0?p[2]:p[3])+p[8];
                    if(fabs(force)>opposing) {
                        double anew=force-copysign(opposing,force);
                        v=anew*remaining;x+=.5*v*remaining;
                    } else v=0;
                } else {x+=.5*(v+next)*dt;v=next;}
            }
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
