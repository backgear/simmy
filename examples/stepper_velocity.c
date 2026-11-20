/* Illustrative low-speed ripple experiments, NOT a calibrated motor.
 * Default: time-series CSV. --sweep: independent speed trials, summary CSV.
 * --point SPEED [HZ]: all six cases at one speed, for convergence checks.
 * See README.md for plotting commands and assumptions.
 */
#include "stepper_experiment.h"
#include <errno.h>
#include <stdlib.h>
#include <string.h>

#ifndef HZ
#define HZ 10000
#endif

static int number(const char *text, double *out) {
  char *end;
  errno = 0;
  *out = strtod(text, &end);
  return !errno && end != text && !*end && isfinite(*out);
}

static void summary(double speed, int hz) {
  double window = StepperSweepWindow(speed);
  for (size_t i = 0; i < STEPPER_SCENARIOS; i++) {
    StepperMetrics m = StepperExperiment(&stepperScenarios[i], speed, hz, 2, window, NULL);
    printf("%s,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
           stepperScenarios[i].name, speed, hz, (double)m.samples/hz, m.meanSpeed,
           m.rmsError, m.acRms, m.peakToPeak, m.maxAbsLoadAngle, m.finalLoadAngle,
           m.reverseFraction, (m.meanSpeed-speed)/speed, m.maxAbsLoadAngle > STEPPER_PI);
  }
}

int main(int argc, char **argv) {
  if (argc == 1) {
    puts("scenario,time_s,position_rad,velocity_rad_s,load_angle_rad,current_a,current_b,torque_nm");
    for (size_t i = 0; i < STEPPER_SCENARIOS; i++)
      StepperExperiment(&stepperScenarios[i], .2, HZ, 2, 2, stdout);
    return 0;
  }
  int sweep = !strcmp(argv[1], "--sweep"), point = !strcmp(argv[1], "--point");
  double low = .01, high = 1.2, points = 121, hz = HZ;
  int valid = 0;
  if (sweep && argc == 2) valid = 1;
  else if (sweep && (argc == 5 || argc == 6)) {
    valid = number(argv[2], &low) && number(argv[3], &high) && number(argv[4], &points);
    if (argc == 6) valid = valid && number(argv[5], &hz);
  } else if (point && (argc == 3 || argc == 4)) {
    valid = number(argv[2], &low);
    high = low;
    if (argc == 4) valid = valid && number(argv[3], &hz);
  }
  /* Bounds keep trials within the intended low-speed experiment and limit
   * memory-free but potentially very long runs from accidental arguments. */
  if (!valid || low < .005 || high > 2 || high < low || points < 2 || points > 1000 ||
      points != floor(points) || hz < 1000 || hz > 100000 || hz != floor(hz)) {
    fprintf(stderr, "Usage: %s [--sweep [MIN MAX POINTS [HZ]] | --point SPEED [HZ]]\n"
                    "Speeds: .005..2 rad/s; points: 2..1000; HZ: 1000..100000.\n", argv[0]);
    return 1;
  }
  puts("scenario,target_rad_s,tick_hz,window_s,mean_rad_s,rms_error_rad_s,ac_rms_rad_s,"
       "peak_to_peak_rad_s,max_abs_load_angle_rad,final_load_angle_rad,reverse_fraction,"
       "relative_mean_error,phase_boundary_crossed");
  int count = point ? 1 : (int)points;
  for (int i = 0; i < count; i++) {
    double speed = point ? low : low*pow(high/low, (double)i/(count-1));
    summary(speed, (int)hz);
  }
  return 0;
}
