/* Ideal true-rotor-flux FOC versus V/Hz, same averaged induction plant.
 * make examples && build/induction_foc > build/induction_foc.csv
 * --steps 8 refines the plant from 25 to 12.5 us with BOTH loop rates fixed.
 */
#include "induction_foc_experiment.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void run(int useFoc, int steps, int stress) {
  FocExperiment t = FocExperimentInit(useFoc);
  const char *name = stress ? "FOC voltage ceiling stress" : (useFoc ? "ideal flux FOC" : "open-loop VHz");
  int ticks = stress ? 6000 : 11000;
  for (int tick = 0; tick < ticks; tick++) {
    double time = tick*INDUCTION_COMMAND_DT;
    double reference = stress ? FocStressReference(time) : FocComparisonReference(time);
    double brake = stress ? 0 : FocComparisonBrake(time);
    FocExperimentTick(&t, reference, brake, steps);
    SimmyInductionOutput output;
    SimmyInductionEvaluate(&t.plant.electrical, &t.plant.motor, &output);
    double flux = hypot(t.plant.electrical.rotorFlux[0], t.plant.electrical.rotorFlux[1]);
    double angle = flux > 1e-8 ? atan2(t.plant.electrical.rotorFlux[1], t.plant.electrical.rotorFlux[0]) : 0;
    double dq[2]; FocPark(output.statorCurrent, angle, dq);
    double voltage = useFoc ? hypot(t.controller.voltage[0], t.controller.voltage[1]) : t.plant.drive.phasePeakV;
    printf("%s,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%d,%d,%d\n",
           name, (tick+1)*INDUCTION_COMMAND_DT, reference, InductionSpeed(&t.plant), brake,
           output.torque, hypot(output.statorCurrent[0], output.statorCurrent[1]), flux,
           voltage, dq[0], dq[1], t.controller.currentReference[0], t.controller.currentReference[1],
           t.controller.fluxReady, t.controller.currentLimited, t.controller.voltageLimited);
  }
  fprintf(stderr, "%s: peak reference current %.4f A, actual current %.4f A, voltage %.4f V; "
                  "voltage saturation %.3f%%\n", name, t.maxReferenceCurrent, t.maxActualCurrent,
                  t.maxVoltage, 100.0*t.voltageLimitedTicks/t.currentTicks);
}

int main(int argc, char **argv) {
  int steps = 4;
  if (argc == 3 && !strcmp(argv[1], "--steps")) {
    char *end; errno = 0;
    long value = strtol(argv[2], &end, 10);
    if (errno || end == argv[2] || *end || value < 1 || value > 1000) {
      fprintf(stderr, "steps must be an integer in 1..1000\n"); return 1;
    }
    steps = (int)value;
  } else if (argc != 1) {
    fprintf(stderr, "Usage: %s [--steps PLANT_STEPS_PER_100US_CURRENT_TICK]\n", argv[0]); return 1;
  }
  puts("drive,time_s,speed_reference_rad_s,rotor_rad_s,brake_nm,torque_nm,current_vector_a,"
       "rotor_flux_wb,voltage_vector_v,id_a,iq_a,id_reference_a,iq_reference_a,"
       "flux_ready,current_reference_limited,voltage_limited");
  run(0, steps, 0); run(1, steps, 0); run(1, steps, 1);
  return 0;
}
