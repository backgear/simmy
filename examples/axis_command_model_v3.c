/* Experimental v3: frozen-v2 moving loss plus one elasto-plastic state.
 * p[0..7]: v2; p[8..11]: positive/negative elastic yield (rad/s^2),
 * stiffness (s^-2), yield velocity scale (rad/s).
 * Yield bounds shrink with speed. Increasing a bound does not create stored
 * force: only displacement loads the spring; projection dissipates its load.
 * Midpoint spring force gives an implicit elastic step. Directional Coulomb
 * loss can balance stored force at rest, so preparation affects restart.
 * Effective command units, NOT calibrated torque. No angle law is assumed.
 * Each capture starts at rest with zero stored force; no maneuver resets.
 */
#include <math.h>
static double clip(double h,double neg,double pos) {return fmax(-neg,fmin(pos,h));}
static double advance(double force,double *v,double *h,double dt,const double *p) {
    double fade=exp(-(*v/p[11])*(*v/p[11]));
    double pos=p[8]*fade,neg=p[9]*fade;
    *h=clip(*h,neg,pos);
    double effective=force-*h;
    int positive=fabs(*v)>1e-10?*v>0:effective>=0;
    double loss=positive?p[2]:p[3],b=positive?p[4]:p[5];
    if(fabs(*v)<1e-10 && effective<=p[2] && effective>=-p[3]) {*v=0;return 0;}
    double drive=force-(positive?loss:-loss)-b*(*v);
    double stiffness=(p[8]==0 && p[9]==0)?0:p[10];
    double a=(drive-*h-.5*stiffness*(*v)*dt)/(1+.25*stiffness*dt*dt);
    double next=*v+a*dt;
    double dx=.5*(*v+next)*dt;
    double trial=*h+stiffness*dx;
    if(trial>pos || trial<-neg) {
        a=drive-.5*(*h+clip(trial,neg,pos));
        next=*v+a*dt;dx=.5*(*v+next)*dt;
    }
    if((*v)*next<0) {
        double tzero=-(*v)/a;
        dx=.5*(*v)*tzero;
        *h=clip(*h+stiffness*dx,neg,pos);*v=0;
        double remaining=dt-tzero;
        /* At most one split: restart from rest cannot cross zero this step. */
        if(remaining>1e-15) dx+=advance(force,v,h,remaining,p);
    } else {
        *v=next;*h=clip(*h+stiffness*dx,neg,pos);
    }
    return dx;
}
void axis_model(const double *u,const int *lengths,int cases,int stride,
                double dt,const double *p,double *xout,double *vout) {
    for(int c=0;c<cases;c++) {
        double x=0,v=0,z=0,h=0;int base=c*stride;
        xout[base]=0;vout[base]=0;
        for(int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1];
            double target=copysign(fmax(fabs(raw)-p[7],0),raw);
            double f=-expm1(-dt/p[6]);
            double avg=target+(z-target)*p[6]/dt*f;z+=(target-z)*f;
            double force=(avg>=0?p[0]:p[1])*avg;
            x+=advance(force,&v,&h,dt,p);
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
