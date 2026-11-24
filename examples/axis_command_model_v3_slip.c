/* Experimental v3b: elastic loading before slip, fixed kinetic loss in slip.
 * p[0..7]: v2. p[8..11]: extra +/- static yield (rad/s^2), spring
 * stiffness (s^-2), fraction [0,1] of moving Coulomb loss in this element.
 * The remainder stays in a parallel set-valued Coulomb element. Yield drops
 * the elastic load to its kinetic value; stopping re-enables elastic loading
 * without clearing that load. Mode changes depend on motion, not commands.
 * This is an effective hypothesis, not identified component friction/torque.
 * Zero extra yield AND zero fraction reproduce v2 (up to roundoff).
 */
#include <math.h>
static double advance(double force,double *v,double *h,int *slipping,double dt,const double *p) {
    double fraction=p[11],cp=p[2]*(1-fraction),cn=p[3]*(1-fraction);
    double kp=p[2]*fraction,kn=p[3]*fraction;
    double yp=kp+p[8],yn=kn+p[9];
    double effective=force-*h;
    int positive=fabs(*v)>1e-10?*v>0:effective>=0;
    double loss=positive?cp:cn,b=positive?p[4]:p[5];
    if(fabs(*v)<1e-10 && effective<=cp && effective>=-cn) {*v=0;*slipping=0;return 0;}
    double stiffness=(p[8]==0 && p[9]==0 && fraction==0)?0:p[10];
    double a,next,dx;
    if(*slipping) {
        *h=positive?kp:-kn;
        a=force-(positive?p[2]:-p[3])-b*(*v);
        next=*v+a*dt;dx=.5*(*v+next)*dt;
    } else {
        double drive=force-(positive?loss:-loss)-b*(*v);
        a=(drive-*h-.5*stiffness*(*v)*dt)/(1+.25*stiffness*dt*dt);
        next=*v+a*dt;dx=.5*(*v+next)*dt;
        double trial=*h+stiffness*dx;
        if(trial>yp || trial<-yn) {
            /* Dissipate the excess stored load on yield. Once slipping,
             * total moving loss is exactly the incumbent directional loss. */
            *slipping=1;*h=positive?kp:-kn;
            a=force-(positive?p[2]:-p[3])-b*(*v);
            next=*v+a*dt;dx=.5*(*v+next)*dt;
        }
    }
    if((*v)*next<0) {
        double tzero=-(*v)/a;dx=.5*(*v)*tzero;
        if(!*slipping)*h+=stiffness*dx;
        *v=0;*slipping=0;
        double remaining=dt-tzero;
        if(remaining>1e-15)dx+=advance(force,v,h,slipping,remaining,p);
    } else {
        *v=next;
        if(!*slipping)*h+=stiffness*dx;
    }
    return dx;
}
void axis_model(const double *u,const int *lengths,int cases,int stride,
                double dt,const double *p,double *xout,double *vout) {
    for(int c=0;c<cases;c++) {
        double x=0,v=0,z=0,h=0;int slipping=0,base=c*stride;
        xout[base]=0;vout[base]=0;
        for(int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1];
            double target=copysign(fmax(fabs(raw)-p[7],0),raw);
            double f=-expm1(-dt/p[6]);
            double avg=target+(z-target)*p[6]/dt*f;z+=(target-z)*f;
            double force=(avg>=0?p[0]:p[1])*avg;
            x+=advance(force,&v,&h,&slipping,dt,p);
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
