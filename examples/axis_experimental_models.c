/* Offline experimental command models. Not firmware. Common p:
 * [G+,G-,tau,deadband], then
 * kind0 linear: [C+,C-,B+,B-]
 * kind1 quadratic: [C+,C-,Q+,Q-]
 * kind2 speed spline: positive then negative loss at 0,5,15,30,45 rad/s
 * kind3 phase: linear plus [sin1,cos1,sin12,cos12] acceleration
 * kind4 LuGre-like bristle: [C,bias_fraction,B+,B-,stiffness,bristle_damping_s]
 * All outputs mechanical rad/s. Phases are measured initial mechanical radians.
 */
#include <math.h>
static double loss(int kind,int positive,double speed,const double*p) {
    if(kind==2) {
        const double knots[5]={0,5,15,30,45};int a=0;
        while(a<3 && speed>knots[a+1])a++;
        int offset=positive?4:9;
        double d=p[offset+a]+(p[offset+a+1]-p[offset+a])*(speed-knots[a])/(knots[a+1]-knots[a]);
        return fmax(1,d);
    }
    double c=positive?p[4]:p[5],b=positive?p[6]:p[7];
    return c+b*(kind==1?speed*speed:speed);
}
void axis_experimental(const double*u,const int*lengths,const double*phase,
                      int cases,int stride,double dt,const double*p,int kind,
                      double*xout,double*vout) {
    for(int c=0;c<cases;c++) {
        double x=0,v=0,z=0,h=kind==4?p[4]*p[5]:0;int base=c*stride;
        xout[base]=0;vout[base]=0;
        for(int i=1;i<lengths[c];i++) {
            double raw=u[base+i-1],target=copysign(fmax(fabs(raw)-p[3],0),raw);
            double f=-expm1(-dt/p[2]),avg=target+(z-target)*p[2]/dt*f;z+=(target-z)*f;
            double force=(avg>=0?p[0]:p[1])*avg;
            if(kind==3) {
                double phi=phase[c]+x+.5*v*dt;
                force+=p[8]*sin(phi)+p[9]*cos(phi)+p[10]*sin(12*phi)+p[11]*cos(12*phi);
            }
            if(kind==4) {
                force+=p[4]*p[5];
                double newh=h;
                if(fabs(v)>1e-14) {
                    double equilibrium=copysign(p[4],v);
                    newh=equilibrium+(h-equilibrium)*exp(-p[8]*fabs(v)*dt/p[4]);
                }
                double friction=.5*(h+newh)+p[9]*(newh-h)/dt+(v>=0?p[6]:p[7])*v;
                double next=v+(force-friction)*dt;x+=.5*(v+next)*dt;v=next;h=newh;
            } else {
                int positive=fabs(v)>1e-10?v>0:force>=0;
                double a=force-copysign(loss(kind,positive,fabs(v),p),positive?1:-1);
                if(fabs(v)<1e-10 && fabs(force)<=loss(kind,force>=0,0,p))v=0;
                else {
                    double next=v+a*dt;
                    if(v*next<0) {
                        double tzero=-v/a;x+=.5*v*tzero;double remain=dt-tzero;
                        double opposing=loss(kind,force>=0,0,p);
                        if(fabs(force)>opposing) {
                            v=(force-copysign(opposing,force))*remain;x+=.5*v*remain;
                        } else v=0;
                    } else {x+=.5*(v+next)*dt;v=next;}
                }
            }
            xout[base+i]=x;vout[base+i]=v;
        }
    }
}
