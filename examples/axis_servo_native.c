/* Native integration checkpoint, NOT bench validation. Links the unmodified
 * analog-servo Autotune.c and Control.c. Encoder-delta feedback only, never
 * simulator velocity truth. One continuous plant state through relay/coast/PI.
 */
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "Autotune.h"
#include "Control.h"
#include "axis_command_step.h"
static const double counts_per_rad=4096.0/6.28318530717958647692;
static int32_t previous_count;
static uint32_t tick;
static int32_t feedback(AxisCommandState*s,int32_t *position) {
    *position=(int32_t)llround(s->x*counts_per_rad);
    int32_t velocity=*position-previous_count;previous_count=*position;return velocity;
}
static void advance(AxisCommandState*s,const double*p,int32_t effort,const char*phase,
                    int32_t target,int32_t position,int32_t velocity) {
    printf("%u,%s,%d,%d,%d,%d,%.17g,%.17g\n",tick++,phase,target,position,velocity,effort,s->x,s->v);
    axis_command_step(s,effort/32767.0,.0005,p);axis_command_step(s,effort/32767.0,.0005,p);
}
int main(int argc,char**argv) {
    if(argc!=14) {fprintf(stderr,"usage: driver amplitude max_ticks distance hold_ticks torque_limit Gp Gn Cp Cn Bp Bn tau deadband\n");return 2;}
    int amplitude=atoi(argv[1]),max_ticks=atoi(argv[2]),distance=atoi(argv[3]),hold=atoi(argv[4]),limit=atoi(argv[5]);
    double p[8];for(int i=0;i<8;i++)p[i]=strtod(argv[i+6],NULL);
    if(amplitude<1 || amplitude>6553 || max_ticks<1 || max_ticks>20000 || hold<1 || hold>10000 || limit<amplitude || limit>6553 || p[6]<=0)return 2;
    AxisCommandState s={0};AutotuneInit();ControlInit(NULL);AutotuneStartRelay(amplitude,(uint32_t)max_ticks);
    puts("tick,phase,reference_count,position_count,velocity_count_per_tick,effort_code,position_rad,velocity_rad_s");
    while(AutotuneIsActive()) {
        int32_t position;int32_t velocity=feedback(&s,&position);
        int32_t effort=AutotuneTick(velocity);advance(&s,p,effort,"relay",0,position,velocity);
    }
    int result=(int)AutotuneGetResult();
    if(result==AUTOTUNE_RESULT_OK) {
        for(int i=0;i<500;i++) {
            int32_t position,velocity=feedback(&s,&position);advance(&s,p,0,"coast",position,position,velocity);
        }
        int32_t anchor=(int32_t)llround(s.x*counts_per_rad);
        ControlGains gains={.posGainQ8=AutotuneGetPosPResultQ8(),.velGainKpQ8=AutotuneGetKpResultQ8(),
            .velGainKiQ8=AutotuneGetKiResultQ8(),.velocityClamp=200,
            .torqueClampMin=-limit,.torqueClampMax=limit,.velocityFilterShift=0};
        ControlSetGains(&gains);
        for(int i=0;i<2*hold;i++) {
            int32_t position,velocity=feedback(&s,&position),target=anchor+(i<hold?distance:0);
            int32_t effort=ControlTick(target,position,velocity,32000);
            advance(&s,p,effort,"control",target,position,velocity);
        }
    }
    fprintf(stderr,"{\"simulated_only\":true,\"result\":%d,\"period_ticks\":%u,\"position_kp_q8\":%d,\"velocity_kp_q8\":%d,\"velocity_ki_q8\":%d,\"ticks\":%u}\n",
        result,AutotuneGetMeasuredPeriodTicks(),AutotuneGetPosPResultQ8(),AutotuneGetKpResultQ8(),AutotuneGetKiResultQ8(),tick);
    return 0;
}
