/* The one C file this tool adds. Batches a whole open-loop replay into a
 * single call so Python never pays per-tick ctypes overhead -- captured
 * axis episodes run into the hundreds of thousands of ticks, and every
 * existing fitter in this repo already solves this the same way (one C
 * call per scipy.optimize.least_squares iteration, not one per tick).
 *
 * Deliberately does not touch src/simmy_config.c's JSON loader or any
 * motor model -- characterization_fitter builds its own SimmyChainParams
 * directly from its fit-spec, and only ever drives bodies[0] with
 * recorded/measured effort (see SimmyChainStep's own contract in
 * simmy.h), so there is no controller and no function pointer to marshal
 * across this boundary. */

#include "simmy.h"

void ChainFitSimulate(const SimmyChainParams *params, const SimmyChainState *initial,
                       const double *force, int steps, double dtTicks,
                       double *outPositions /* bodyCount x steps, body-major */) {
  SimmyChainState state = *initial;
  int bodyCount = params->bodyCount;
  for (int i = 0; i < steps; i++) {
    SimmyChainStepDt(&state, params, force[i], dtTicks);
    for (int b = 0; b < bodyCount; b++) {
      outPositions[(size_t)b * (size_t)steps + (size_t)i] = state.bodies[b].position;
    }
  }
}
