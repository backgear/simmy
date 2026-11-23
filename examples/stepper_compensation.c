/* Controlled comparison: same motor/drive, four current-domain policies.
 * No extra mechanical force is injected. See docs/motors/stepper.md.
 */
#include "stepper_experiment.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

typedef struct { const char *name; int feedforward, damping; } Policy;
static const Policy policies[] = {
  {"none", 0, 0}, {"detent feedforward", 1, 0},
  {"active damping", 0, 1}, {"both", 1, 1}
};
static const int scenarios[] = {3, 1, 5}; /* detent, microsteps, combined */
static const char *keys[] = {"detent", "microsteps", "combined"};

static int number(const char *text, double *out) {
  char *end;
  errno = 0;
  *out = strtod(text, &end);
  return !errno && end != text && !*end && isfinite(*out);
}

static void run(double speed, int hz, int onlyScenario, int trace) {
  for (int scenario = 0; scenario < 3; scenario++) {
    if (onlyScenario >= 0 && scenario != onlyScenario) continue;
    for (int policy = 0; policy < 4; policy++) {
      StepperCompensation c = {.detentEstimate=policies[policy].feedforward ? .012 : 0,
          .detentPhaseEstimate=.3, .detentCyclesEstimate=200,
          .damping=policies[policy].damping ? .08 : 0,
          .torqueLimit=.2, .phaseCurrentLimit=1.5};
      /* Both detent-present scenarios use the same exact known table.
       * The pure microstep ablation has no detent, so its table is zero. */
      if (scenario == 1) c.detentEstimate = 0;
      StepperScenario s = stepperScenarios[scenarios[scenario]];
      const char *name = s.name;
      if (trace) s.name = policies[policy].name;
      StepperMetrics m = StepperExperimentControlled(&s, speed, hz, 2,
                                     trace ? 2 : StepperSweepWindow(speed), trace ? stdout : NULL, &c);
      if (!trace)
        printf("%s,%s,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
               name, policies[policy].name, speed, hz, (double)m.samples/hz, m.meanSpeed,
               m.rmsError, m.acRms, m.reverseFraction, m.maxAbsLoadAngle,
               m.currentVectorRms, m.peakPhaseCurrent, m.correctionRms,
               m.torqueLimitFraction, m.currentLimitFraction, m.maxAbsLoadAngle > STEPPER_PI);
    }
  }
}

int main(int argc, char **argv) {
  double speed = .2, hz = 10000;
  int sweep = argc >= 2 && !strcmp(argv[1], "--sweep");
  int point = argc >= 2 && !strcmp(argv[1], "--point");
  int trace = argc >= 2 && !strcmp(argv[1], "--trace");
  int valid = 0, onlyScenario = -1;
  if (sweep && (argc == 2 || argc == 3))
    valid = argc == 2 || number(argv[2], &hz);
  if (point && (argc == 3 || argc == 4)) {
    valid = number(argv[2], &speed);
    if (argc == 4) valid = valid && number(argv[3], &hz);
  }
  if (trace && (argc == 4 || argc == 5)) {
    for (int i = 0; i < 3; i++) if (!strcmp(argv[2], keys[i])) onlyScenario = i;
    valid = onlyScenario >= 0 && number(argv[3], &speed);
    if (argc == 5) valid = valid && number(argv[4], &hz);
  }
  if (!valid || speed < .005 || speed > 2 || hz < 1000 || hz > 100000 || hz != floor(hz)) {
    fprintf(stderr, "Usage: %s --sweep [HZ] | --point SPEED [HZ] | "
                    "--trace detent|microsteps|combined SPEED [HZ]\n", argv[0]);
    return 1;
  }
  if (trace)
    puts("policy,time_s,position_rad,velocity_rad_s,load_angle_rad,current_a,current_b,torque_nm");
  else
    puts("scenario,policy,target_rad_s,tick_hz,window_s,mean_rad_s,rms_error_rad_s,ac_rms_rad_s,"
         "reverse_fraction,max_abs_load_angle_rad,current_vector_rms_a,peak_phase_current_a,"
         "correction_rms_nm,torque_limit_fraction,current_limit_fraction,phase_boundary_crossed");
  if (sweep) {
    for (int i = 0; i < 61; i++) run(.01*pow(120, i/60.0), (int)hz, -1, 0);
  } else run(speed, (int)hz, onlyScenario, trace);
  return 0;
}
