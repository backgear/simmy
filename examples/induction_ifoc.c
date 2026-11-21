/* Encoder-assisted indirect FOC, true-flux oracle and parameter mismatch.
 * Only scoring code below reads true flux for the indirect variants.
 * build/induction_ifoc > build/induction_ifoc.csv
 */
#include "induction_foc_experiment.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void run(const char *name, int indirect, double rrScale, int steps) {
  FocExperiment t = indirect ? IfocExperimentInit(rrScale) : FocExperimentInit(1);
  for (int tick = 0; tick < 11000; tick++) {
    double time = tick*.001, reference = FocComparisonReference(time), brake = IfocComparisonBrake(time);
    FocExperimentTick(&t, reference, brake, steps);
    SimmyInductionOutput output;
    SimmyInductionEvaluate(&t.plant.electrical, &t.plant.motor, &output);
    /* Truth is an evaluation oracle, never an input to indirect control. */
    double trueFlux = hypot(t.plant.electrical.rotorFlux[0], t.plant.electrical.rotorFlux[1]);
    double trueAngle = atan2(t.plant.electrical.rotorFlux[1], t.plant.electrical.rotorFlux[0]);
    double controlFlux = indirect ? t.estimate.fluxMagnitude : trueFlux;
    double controlAngle = indirect ? t.estimate.fluxAngle : trueAngle;
    int valid = trueFlux > .05 && controlFlux > .05;
    double error = valid ? remainder(controlAngle-trueAngle, 2*FOC_PI)*180/FOC_PI : 0;
    double dq[2]; FocPark(output.statorCurrent, controlAngle, dq);
    double predictedTorque = 1.5*t.params.polePairs*t.params.lm/t.params.lr*controlFlux*dq[1];
    printf("%s,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%.9f,%.9f,%.9f,%.9f,%d\n",
           name, rrScale, (tick+1)*.001, reference, InductionSpeed(&t.plant), brake,
           trueFlux, controlFlux, error, valid, output.torque, predictedTorque,
           hypot(output.statorCurrent[0], output.statorCurrent[1]),
           hypot(t.controller.voltage[0], t.controller.voltage[1]), t.controller.fluxReady);
  }
  fprintf(stderr, "%s: actual peak current %.4f A; max reference %.4f A; max voltage %.4f V\n",
          name, t.maxActualCurrent, t.maxReferenceCurrent, t.maxVoltage);
}

int main(int argc, char **argv) {
  int steps = 4;
  if (argc == 3 && !strcmp(argv[1], "--steps")) {
    char *end; errno = 0; long value = strtol(argv[2], &end, 10);
    if (errno || end == argv[2] || *end || value < 1 || value > 1000) {
      fprintf(stderr, "steps must be an integer in 1..1000\n"); return 1;
    }
    steps = (int)value;
  } else if (argc != 1) {
    fprintf(stderr, "Usage: %s [--steps PLANT_STEPS_PER_100US_CURRENT_TICK]\n", argv[0]); return 1;
  }
  puts("controller,controller_rr_scale,time_s,reference_rad_s,speed_rad_s,brake_nm,"
       "true_flux_wb,control_flux_wb,angle_error_deg,angle_valid,torque_nm,"
       "control_torque_nm,current_vector_a,voltage_vector_v,flux_ready");
  run("true-flux oracle", 0, 1, steps);
  run("indirect matched", 1, 1, steps);
  run("indirect Rr x0.5", 1, .5, steps);
  run("indirect Rr x1.5", 1, 1.5, steps);
  return 0;
}
