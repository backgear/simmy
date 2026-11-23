/* Offline command-to-motion model kernel. SI mechanical radians/seconds.
 * p: gain,bias,kinetic,extra_static,stribeck_speed,viscous,tau,deadband.
 * Drive filter has exact interval-average input. Mechanics uses trapezoidal
 * position integration and a velocity-zero crossing/stiction condition.
 * No electrical phase model, position cogging, or measured-current input.
 */
#include <math.h>
void axis_model(const double *u, const int *lengths, int cases, int stride,
                double dt, const double *p, double *xout, double *vout) {
    for (int c=0;c<cases;c++) {
        double x=0,v=0,z=0;
        int base=c*stride;
        xout[base]=0;vout[base]=0;
        for (int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1];
            double target=copysign(fmax(fabs(raw)-p[7],0),raw);
            double f=-expm1(-dt/p[6]);
            double avg=target+(z-target)*p[6]/dt*f;
            z+=(target-z)*f;
            double force=p[0]*avg+p[1];
            double friction=p[2]+p[3]*exp(-(v/p[4])*(v/p[4]));
            double static_friction=p[2]+p[3];
            double direction=fabs(v)>1e-10?copysign(1,v):copysign(1,force);
            double a=force-direction*friction-p[5]*v;
            if (fabs(v)<1e-10 && fabs(force)<=static_friction) {
                v=0;
            } else {
                double next=v+a*dt;
                if (v*next<0) {
                    double tzero=-v/a;
                    x+=.5*v*tzero;
                    double remain=dt-tzero;
                    if (fabs(force)>static_friction) {
                        double anew=force-copysign(static_friction,force);
                        v=anew*remain;x+=.5*v*remain;
                    } else v=0;
                } else {x+=.5*(v+next)*dt;v=next;}
            }
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
