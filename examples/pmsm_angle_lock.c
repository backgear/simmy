/* Forced-alignment / angle-lock torque pulse: a fixed stator voltage
 * vector pulls the PM rotor into alignment like a magnetic pendulum.
 * No controller, no feedback -- see examples/pmsm_angle_lock.h for the
 * restoring-torque derivation and why this is a validation of the PLANT,
 * not of any control algorithm. Sweeps rotor inertia across two orders
 * of magnitude to show the textbook period ~ sqrt(J) scaling and a
 * common equilibrium, all from the same restoring-torque law.
 *
 * make examples && build/pmsm_angle_lock > build/pmsm_angle_lock.csv
 */
#include "pmsm_angle_lock.h"
#include <stdio.h>

#define DISPLAY_DT .001
#define TICKS_PER_DISPLAY 10 /* DISPLAY_DT / PMSM_ANGLE_LOCK_DT */

int main(void) {
  double inertiasKgM2[] = {.0002, .001, .005, .025};
  double voltageMagnitude = 5.0;    /* fixed drive vector, volts */
  double driveAngle = 0.0;          /* fixed drive electrical angle, rad */
  double durationSeconds = 8.0;

  puts("inertia_kgm2,time_s,mechanical_angle_rad,speed_rad_s,torque_nm,current_a");
  for (size_t i = 0; i < sizeof(inertiasKgM2)/sizeof(inertiasKgM2[0]); i++) {
    double inertia = inertiasKgM2[i];
    PmAngleLockTrial t = PmAngleLockTrialInit(driveAngle, voltageMagnitude, inertia, 0);
    /* 90 electrical degrees displaced from the equilibrium, at rest --
     * same starting condition for every inertia, so the only thing that
     * differs between curves is J. */
    double equilibrium = PmAngleLockEquilibrium(driveAngle, &t.motor);
    double start = equilibrium + (PMSM_ANGLE_LOCK_PI/2)/t.motor.polePairs;
    PmAngleLockSetState(&t, start, 0);

    int displayTicks = (int)(durationSeconds/DISPLAY_DT);
    for (int tick = 0; tick < displayTicks; tick++) {
      SimmyPmOutput output = {0};
      for (int sub = 0; sub < TICKS_PER_DISPLAY; sub++) {
        if (!PmAngleLockTick(&t, &output)) {
          fprintf(stderr, "PmAngleLockTick: invalid state/parameters\n");
          return 1;
        }
      }
      printf("%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", inertia, (tick+1)*DISPLAY_DT,
             PmAngleLockAngle(&t), PmAngleLockSpeed(&t), output.torque,
             hypot(t.electrical.current[0], t.electrical.current[1]));
    }
    fprintf(stderr, "J=%.4f kg*m^2: final angle %.6f rad (equilibrium %.6f), "
                    "final speed %.6f rad/s\n", inertia, PmAngleLockAngle(&t), equilibrium,
            PmAngleLockSpeed(&t));
  }
  return 0;
}
