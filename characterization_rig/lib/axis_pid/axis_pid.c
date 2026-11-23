#include "axis_pid.h"
#include "Control.h"
static axis_pid_config_t config;
static int32_t previous;
static bool valid;
bool axis_pid_validate(const axis_pid_config_t *c) {
    return c && c->position_kp_q8>0 && c->position_kp_q8<=256
        && c->velocity_kp_q8>0 && c->velocity_kp_q8<=1048576
        && c->velocity_ki_q8>=0 && c->velocity_ki_q8<=1048576
        && c->torque_limit_code>0 && c->torque_limit_code<=6553
        && c->error_limit_counts>0 && c->error_limit_counts<=4096
        && c->velocity_clamp>0 && c->velocity_clamp<=32;
}
bool axis_pid_init(const axis_pid_config_t *c,int32_t position) {
    valid=axis_pid_validate(c);if(!valid)return false;
    config=*c;previous=position;
    ControlGains gains={.posGainQ8=c->position_kp_q8,.velGainKpQ8=c->velocity_kp_q8,
        .velGainKiQ8=c->velocity_ki_q8,.velocityClamp=c->velocity_clamp,
        .torqueClampMin=-c->torque_limit_code,.torqueClampMax=c->torque_limit_code};
    ControlInit(&gains);return true;
}
bool axis_pid_tick(int32_t reference,int32_t position,int32_t *effort) {
    *effort=0;
    int64_t error=(int64_t)reference-position,velocity=(int64_t)position-previous;
    if(!valid || error>config.error_limit_counts || error<-config.error_limit_counts
        || velocity>64 || velocity<-64) {valid=false;return false;}
    previous=position;
    *effort=ControlTick(reference,position,(int32_t)velocity,72000);
    return true;
}
