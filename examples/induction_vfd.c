/* Averaged open-loop induction motor experiments, no rotor feedback to VFD.
 * make examples && build/induction_vfd > build/induction_vfd.csv
 * build/induction_vfd --steps 80 refines integration ONLY, not drive updates.
 */
#include "induction_experiment.h"
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void run(const char *name, double frequency, double boost, double ramp, int steps) {
  InductionTrial t = InductionTrialInit(boost, ramp);
  int lowSpeed = frequency == 5;
  int ticks = lowSpeed ? 4000 : 11000;
  for (int tick = 0; tick < ticks; tick++) {
    /* Passive brake opposes motion and can hold a stalled shaft. This is
     * NOT a gravity load that would back-drive the machine after stall. */
    double load = lowSpeed ? (tick >= 1000 ? 3 : 0) :
        (tick < 3000 ? 0 : (tick < 5000 || tick >= 8000 ? 6 : 20));
    t.chain.bodies[0].coulombLevel = t.chain.bodies[0].stictionLimit = load;
    InductionTrialTick(&t, frequency, steps);
    SimmyInductionOutput o;
    SimmyInductionEvaluate(&t.electrical, &t.motor, &o);
    double speed = InductionSpeed(&t);
    double sync = 2*INDUCTION_PI*t.drive.frequencyHz/t.motor.polePairs;
    double slipHz = t.drive.frequencyHz-t.motor.polePairs*speed/(2*INDUCTION_PI);
    printf("%s,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
           name, (tick+1)*INDUCTION_COMMAND_DT, t.drive.frequencyHz, t.drive.phasePeakV,
           sync, speed, slipHz, load, o.torque, hypot(o.statorCurrent[0], o.statorCurrent[1]),
           hypot(t.electrical.rotorFlux[0], t.electrical.rotorFlux[1]), o.magneticEnergy,
           t.mechanics.bodies[0].position*t.mount.radiansPerCount);
  }
}

int main(int argc, char **argv) {
  int steps = 40;
  if (argc == 3 && !strcmp(argv[1], "--steps")) {
    char *end;
    errno = 0;
    long value = strtol(argv[2], &end, 10);
    if (errno || end == argv[2] || *end || value < 1 || value > 1000) {
      fprintf(stderr, "steps must be an integer in 1..1000\n"); return 1;
    }
    steps = (int)value;
  } else if (argc != 1) {
    fprintf(stderr, "Usage: %s [--steps INTEGRATION_STEPS_PER_1MS_COMMAND]\n", argv[0]); return 1;
  }
  puts("scenario,time_s,frequency_hz,phase_peak_v,synchronous_rad_s,rotor_rad_s,slip_hz,"
       "brake_nm,torque_nm,stator_current_vector_a,rotor_flux_wb,magnetic_energy_j,position_rad");
  run("ramped 50 Hz", 50, 0, 25, steps);
  run("direct 50 Hz", 50, 0, 0, steps);
  run("5 Hz no boost", 5, 0, 10, steps);
  run("5 Hz boost", 5, 10, 10, steps);
  return 0;
}
