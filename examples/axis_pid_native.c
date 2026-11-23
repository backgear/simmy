/* Native backend for the SAME PC tuner and SAME MCU controller sources.
 * Continuous frozen-plant state across trials; encoder quantization is the
 * only control feedback. No torque replay, truth velocity, or maneuver resets.
 */
#include <stdint.h>
#include "axis_pid.h"
#include "axis_command_step.h"
int axis_pid_native(AxisCommandState *s,const double *p,const axis_pid_config_t *cfg,
                    const int32_t *refs,int count,int period_ms,int pre_ms,int post_ms,double *out) {
    int total=2*(pre_ms+count*period_ms+post_ms)+1;
    const double cpr=4096.0/6.28318530717958647692;
    int32_t origin=(int32_t)llround(s->x*cpr),effort=0,reference=0;
    int initialized=0;
    if(!axis_pid_validate(cfg) || period_ms<1 || count<2)return -1;
    for(int i=0;i<total;i++) {
        int32_t position=(int32_t)llround(s->x*cpr)-origin;
        if(i%2==0) {
            int ms=i/2;
            if(ms>=pre_ms && ms<pre_ms+count*period_ms) {
                if(!initialized) {axis_pid_init(cfg,position);initialized=1;}
                reference=refs[(ms-pre_ms)/period_ms];
                if(!axis_pid_tick(reference,position,&effort))return i+1;
            } else effort=0;
        }
        out[5*i]=position;out[5*i+1]=effort;out[5*i+2]=reference;
        out[5*i+3]=s->x;out[5*i+4]=s->v;
        if(i+1<total)axis_command_step(s,effort/32767.0,.0005,p);
    }
    return 0;
}
