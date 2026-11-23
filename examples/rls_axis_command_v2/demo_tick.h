/* Diagnostic demo sampling: one encoder observation and one controller call
 * per 1ms tick. The effort recorded at t drives the following plant interval.
 * Legacy/no-reference switches isolate old harness errors; never use them as
 * claims about real hardware. Requires the helpers defined by main.c.
 */
#ifndef AXIS_DEMO_TICK_H
#define AXIS_DEMO_TICK_H

typedef struct {
  int previousPosition, initialized, tick, first;
  int encoderFeedback, velocityReference;
} AxisDemo;

static int64_t DemoClamp(int64_t value, int64_t lo, int64_t hi) {
  return value < lo ? lo : value > hi ? hi : value;
}

static void DemoTick(FILE *out, AxisDemo *d, AxisCommandState *plant,
                     const double *p, double countsPerRad, int32_t target,
                     int32_t requestedVelocity, const char *phase) {
  int32_t position = PlantPositionCounts(plant, countsPerRad);
  int32_t delta = d->initialized ? position-d->previousPosition : 0;
  d->previousPosition=position;d->initialized=1;
  int32_t truthVelocity=PlantVelocityCountsPerTick(plant,countsPerRad);
  int32_t velocity=d->encoderFeedback ? delta : truthVelocity;
  int32_t appliedReference=d->velocityReference ? requestedVelocity : 0;
  const ControlGains *g=ControlGetGains();
  int64_t limit=(int64_t)g->velocityClamp*256;
  int64_t referenceQ8=DemoClamp(((int64_t)target-position)*g->posGainQ8
                               +(int64_t)appliedReference*256,-limit,limit);
  int64_t errorQ8=referenceQ8-(int64_t)velocity*256;
  int64_t before=ControlGetVelocityIntegralQ16();
  int32_t torque=ControlTickWithVelocity(target,position,velocity,appliedReference,32000);
  int64_t after=ControlGetVelocityIntegralQ16();
  /* These decompositions apply to this demo's unfiltered, zero-deadband,
   * zero-acceleration-FF configuration. Do not infer measured torque from them.
   * Output minus stored I equals P only away from clipping/anti-windup. */
  double proportional=(double)(errorQ8*g->velGainKpQ8)/65536.0;
  int64_t candidate=DemoClamp(before+errorQ8*g->velGainKiQ8,
                            (int64_t)g->torqueClampMin*65536,
                            (int64_t)g->torqueClampMax*65536);
  double raw=proportional+(double)candidate/65536.0;
  if(!d->first)fprintf(out,",\n");
  d->first=0;
  fprintf(out,"{\"t\":%.6f,\"seq\":%d,\"position\":%d,\"velocity\":%d,"
              "\"encoder_delta\":%d,\"rounded_truth_velocity\":%d,\"torque\":%d,"
              "\"cmd_position\":%d,\"cmd_velocity\":%d,\"applied_velocity_reference\":%d,"
              "\"fault_latched\":false,\"phase\":\"%s\",\"integral\":%.9f,"
              "\"integral_before\":%.9f,\"proportional\":%.9f,"
              "\"velocity_error_q8\":%lld,\"unclamped_candidate_torque\":%.9f,"
              "\"torque_minus_stored_integral\":%.9f}",
          (double)d->tick/CONTROL_TICK_HZ,d->tick,position,velocity,delta,truthVelocity,torque,
          target,requestedVelocity,appliedReference,phase,(double)after/65536.0,
          (double)before/65536.0,proportional,(long long)errorQ8,raw,torque-(double)after/65536.0);
  d->tick++;
  StepPlant(plant,torque,p);
}
#endif
