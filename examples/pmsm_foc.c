/* Continuous-angle (resolver-style) FOC on simmy's existing sine-PM plant.
 * Reference harness: the point is to give an EXTERNAL no-Hall, FOC-
 * commutated servo-amp firmware (e.g. an AMC SX25A-class drive) something
 * to be tested against, the same way docs/axis_controller_development.md
 * already lets a real axis controller tune against a simulated motor.
 * See examples/pmsm_foc.h for the controller/trial contract and
 * docs/motors/bldc.md for the plant.
 *
 * make examples && build/pmsm_foc > build/pmsm_foc.csv
 */
#include "pmsm_foc.h"
#include <stdio.h>

static void run(const char *name, int stress, int ticks) {
  PmFocTrial t = PmFocTrialInit();
  for (int tick = 0; tick < ticks; tick++) {
    double time = tick*.001;
    double reference = stress ? PmFocStressReference(time) : PmFocComparisonReference(time);
    double brake = stress ? 0 : PmFocComparisonBrake(time);
    PmFocTrialTick(&t, reference, brake);
    double mechanicalAngle = PmFocTrialAngle(&t);
    double idealElectricalAngle = remainder(
        t.motor.polePairs*mechanicalAngle+t.motor.electricalOffset, 2*FOC_PI);
    double sensedAngle = PmResolverSensedAngle(mechanicalAngle, t.motor.polePairs,
                                               t.motor.electricalOffset, &t.resolver);
    double angleErrorDeg = remainder(sensedAngle-idealElectricalAngle, 2*FOC_PI)*180/FOC_PI;
    printf("%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%.4f\n",
           name, (tick+1)*.001, reference, PmFocTrialSpeed(&t), brake,
           hypot(t.electrical.current[0], t.electrical.current[1]),
           hypot(t.controller.voltage[0], t.controller.voltage[1]),
           t.controller.measuredCurrent[0], t.controller.measuredCurrent[1],
           t.controller.currentReference[0], t.controller.currentReference[1],
           t.controller.currentLimited, t.controller.voltageLimited, angleErrorDeg);
  }
  fprintf(stderr, "%s: peak reference current %.4f A; actual current %.4f A; voltage %.4f V; "
                  "voltage saturation %.3f%%\n", name, t.maxReferenceCurrent, t.maxActualCurrent,
          t.maxVoltage, 100.0*t.voltageLimitedTicks/t.currentTicks);
}

int main(void) {
  puts("drive,time_s,speed_reference_rad_s,speed_rad_s,brake_nm,current_vector_a,"
       "voltage_vector_v,id_a,iq_a,id_reference_a,iq_reference_a,"
       "current_reference_limited,voltage_limited,sensed_angle_error_deg");
  run("PMSM FOC", 0, 5000);
  run("PMSM FOC voltage ceiling stress", 1, 4000);
  return 0;
}
