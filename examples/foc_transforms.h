#ifndef FOC_TRANSFORMS_H
#define FOC_TRANSFORMS_H

/* Generic amplitude-invariant 2-axis Park / inverse-Park and a symmetric
 * vector clamp. Pure math on a pair of doubles -- nothing here is specific
 * to any one plant (induction, PMSM, ...), so every FOC-style example
 * controller in this directory shares this header instead of redefining
 * its own copy. */
#include <math.h>

#define FOC_PI 3.14159265358979323846

static inline void FocPark(const double ab[2], double angle, double dq[2]) {
  double c = cos(angle), s = sin(angle);
  dq[0] = c*ab[0]+s*ab[1];
  dq[1] = -s*ab[0]+c*ab[1];
}

static inline void FocInversePark(const double dq[2], double angle, double ab[2]) {
  double c = cos(angle), s = sin(angle);
  ab[0] = c*dq[0]-s*dq[1];
  ab[1] = s*dq[0]+c*dq[1];
}

static inline double FocClamp(double value, double limit) {
  return fmax(-limit, fmin(limit, value));
}
#endif
