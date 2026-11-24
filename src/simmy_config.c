#include "simmy.h"
#include "simmy_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* See docs/build.md for the full config file schema. */

static char *ReadWholeFile(const char *path, char *errMsg, size_t errMsgLen) {
  FILE *f = fopen(path, "rb");
  if (!f) {
    if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "could not open '%s'", path);
    return NULL;
  }
  fseek(f, 0, SEEK_END);
  long size = ftell(f);
  fseek(f, 0, SEEK_SET);
  if (size < 0) {
    fclose(f);
    if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "could not determine size of '%s'", path);
    return NULL;
  }
  char *buf = (char *)malloc((size_t)size + 1);
  size_t readCount = fread(buf, 1, (size_t)size, f);
  fclose(f);
  buf[readCount] = '\0';
  return buf;
}

static void FillBodyParams(SimmyBodyParams *out, const SimmyJsonValue *obj) {
  SimmyBodyParams zero = {0};
  *out = zero;
  out->inertia = SimmyJsonNumber(SimmyJsonObjectGet(obj, "inertia"), 0.0);
  out->friction = SimmyJsonNumber(SimmyJsonObjectGet(obj, "friction"), 0.0);
  out->stictionLimit = SimmyJsonNumber(SimmyJsonObjectGet(obj, "stictionLimit"), 0.0);
  out->coulombLevel = SimmyJsonNumber(SimmyJsonObjectGet(obj, "coulombLevel"), 0.0);
  out->gravityBias = SimmyJsonNumber(SimmyJsonObjectGet(obj, "gravityBias"), 0.0);
  out->rippleAmplitude = SimmyJsonNumber(SimmyJsonObjectGet(obj, "rippleAmplitude"), 0.0);
  out->ripplePeriodCounts = SimmyJsonNumber(SimmyJsonObjectGet(obj, "ripplePeriodCounts"), 0.0);
  out->frictionBumpCenter = SimmyJsonNumber(SimmyJsonObjectGet(obj, "frictionBumpCenter"), 0.0);
  out->frictionBumpHalfWidth = SimmyJsonNumber(SimmyJsonObjectGet(obj, "frictionBumpHalfWidth"), 0.0);
  out->frictionBumpExtra = SimmyJsonNumber(SimmyJsonObjectGet(obj, "frictionBumpExtra"), 0.0);
}

static void FillCouplingParams(SimmyCouplingParams *out, const SimmyJsonValue *obj) {
  SimmyCouplingParams zero = {0};
  *out = zero;
  out->springK = SimmyJsonNumber(SimmyJsonObjectGet(obj, "springK"), 0.0);
  out->springD = SimmyJsonNumber(SimmyJsonObjectGet(obj, "springD"), 0.0);
  out->backlashGap = SimmyJsonNumber(SimmyJsonObjectGet(obj, "backlashGap"), 0.0);
  out->forceLimit = SimmyJsonNumber(SimmyJsonObjectGet(obj, "forceLimit"), 0.0);
}

int SimmyChainParamsFromJsonFile(const char *path, SimmyChainParams *out,
                                  char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyChainParams zero = {0};
  *out = zero;

  const SimmyJsonValue *bodies = SimmyJsonObjectGet(root, "bodies");
  int n = SimmyJsonArrayLength(bodies);
  if (n <= 0) {
    if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "'bodies' must be a non-empty array");
    SimmyJsonFree(root);
    return 1;
  }
  if (n > SIMMY_MAX_BODIES) n = SIMMY_MAX_BODIES;
  out->bodyCount = n;
  for (int i = 0; i < n; i++) {
    FillBodyParams(&out->bodies[i], SimmyJsonArrayGet(bodies, i));
    if (out->bodies[i].inertia <= 0.0) {
      if (errMsg && errMsgLen) {
        snprintf(errMsg, errMsgLen, "bodies[%d].inertia must be > 0 (required, no default)", i);
      }
      SimmyJsonFree(root);
      return 1;
    }
  }

  const SimmyJsonValue *couplings = SimmyJsonObjectGet(root, "couplings");
  int cn = SimmyJsonArrayLength(couplings);
  if (cn > n - 1) cn = n - 1;
  for (int i = 0; i < cn; i++) FillCouplingParams(&out->couplings[i], SimmyJsonArrayGet(couplings, i));

  out->feedbackBody = (int)SimmyJsonNumber(SimmyJsonObjectGet(root, "feedbackBody"), 0.0);
  if (out->feedbackBody < 0 || out->feedbackBody >= n) out->feedbackBody = 0;
  out->subSteps = (int)SimmyJsonNumber(SimmyJsonObjectGet(root, "subSteps"), 0.0);

  SimmyJsonFree(root);
  return 0;
}

static SimmyAnalogStageType StageTypeFromString(const char *s, int *ok) {
  *ok = 1;
  if (strcmp(s, "gain") == 0) return SIMMY_STAGE_GAIN;
  if (strcmp(s, "saturate") == 0) return SIMMY_STAGE_SATURATE;
  if (strcmp(s, "quantize") == 0) return SIMMY_STAGE_QUANTIZE;
  if (strcmp(s, "lag") == 0) return SIMMY_STAGE_LAG;
  if (strcmp(s, "noise") == 0) return SIMMY_STAGE_NOISE;
  *ok = 0;
  return SIMMY_STAGE_GAIN;
}

int SimmyAnalogChainParamsFromJsonFile(const char *path, SimmyAnalogChainParams *out,
                                        char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyAnalogChainParams zero = {0};
  *out = zero;

  const SimmyJsonValue *stages = SimmyJsonObjectGet(root, "analogChain");
  int n = SimmyJsonArrayLength(stages);   /* 0 is fine -- an absent/empty analogChain is a no-op pass-through */
  if (n > SIMMY_MAX_ANALOG_STAGES) n = SIMMY_MAX_ANALOG_STAGES;
  out->stageCount = n;
  for (int i = 0; i < n; i++) {
    const SimmyJsonValue *stage = SimmyJsonArrayGet(stages, i);
    int ok;
    out->stages[i].type = StageTypeFromString(SimmyJsonString(SimmyJsonObjectGet(stage, "type"), ""), &ok);
    if (!ok) {
      if (errMsg && errMsgLen) {
        snprintf(errMsg, errMsgLen, "analogChain[%d].type must be one of "
                 "gain|saturate|quantize|lag|noise", i);
      }
      SimmyJsonFree(root);
      return 1;
    }
    out->stages[i].param0 = SimmyJsonNumber(SimmyJsonObjectGet(stage, "param0"), 0.0);
    out->stages[i].param1 = SimmyJsonNumber(SimmyJsonObjectGet(stage, "param1"), 0.0);
  }

  SimmyJsonFree(root);
  return 0;
}

/* ---- motor parameters ----------------------------------------------------
 * Same hand-rolled parser and "caller-validates" convention as the chain/
 * analog-chain loaders above. Each function opens and parses the file
 * independently, so a single config file can carry a "bodies"/"couplings"/
 * "analogChain" section alongside exactly one motor section -- see
 * docs/build.md. */

static void FillDoubleArray(double *out, int n, const SimmyJsonValue *arr, double defaultValue) {
  for (int i = 0; i < n; i++) out[i] = SimmyJsonNumber(SimmyJsonArrayGet(arr, i), defaultValue);
}

int SimmyStepperParamsFromJsonFile(const char *path, SimmyStepperParams *out,
                                    char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyStepperParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "stepperMotor");
  out->electricalCyclesPerRev = (int)SimmyJsonNumber(SimmyJsonObjectGet(obj, "electricalCyclesPerRev"), 0.0);
  if (out->electricalCyclesPerRev <= 0) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "stepperMotor.electricalCyclesPerRev must be > 0 (required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }
  out->electricalOffset = SimmyJsonNumber(SimmyJsonObjectGet(obj, "electricalOffset"), 0.0);
  out->torqueConstant = SimmyJsonNumber(SimmyJsonObjectGet(obj, "torqueConstant"), 0.0);
  out->detentAmplitude = SimmyJsonNumber(SimmyJsonObjectGet(obj, "detentAmplitude"), 0.0);
  out->detentCyclesPerRev = (int)SimmyJsonNumber(SimmyJsonObjectGet(obj, "detentCyclesPerRev"), 0.0);
  out->detentPhase = SimmyJsonNumber(SimmyJsonObjectGet(obj, "detentPhase"), 0.0);
  if (out->detentAmplitude != 0.0 && out->detentCyclesPerRev <= 0) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen,
               "stepperMotor.detentCyclesPerRev must be > 0 when detentAmplitude != 0");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyStepperCurrentParamsFromJsonFile(const char *path, SimmyStepperCurrentParams *out,
                                           char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyStepperCurrentParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "stepperCurrentPath");
  FillDoubleArray(out->gainError, 2, SimmyJsonObjectGet(obj, "gainError"), 0.0);
  FillDoubleArray(out->offset, 2, SimmyJsonObjectGet(obj, "offset"), 0.0);
  FillDoubleArray(out->quantizeStep, 2, SimmyJsonObjectGet(obj, "quantizeStep"), 0.0);

  SimmyJsonFree(root);
  return 0;
}

int SimmyInductionParamsFromJsonFile(const char *path, SimmyInductionParams *out,
                                      char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyInductionParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "inductionMotor");
  out->polePairs = (int)SimmyJsonNumber(SimmyJsonObjectGet(obj, "polePairs"), 0.0);
  out->statorResistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "statorResistance"), 0.0);
  out->rotorResistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "rotorResistance"), 0.0);
  out->statorLeakageInductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "statorLeakageInductance"), 0.0);
  out->rotorLeakageInductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "rotorLeakageInductance"), 0.0);
  out->magnetizingInductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "magnetizingInductance"), 0.0);
  if (!SimmyInductionParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "inductionMotor parameters invalid: need polePairs > 0, "
               "stator/rotor resistance >= 0, and positive leakage/magnetizing inductance "
               "(statorLeakageInductance, rotorLeakageInductance, magnetizingInductance are "
               "required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

static SimmyWindingConnection ConnectionFromString(const char *s, int *ok) {
  *ok = 1;
  if (strcmp(s, "wye") == 0) return SIMMY_WYE;
  if (strcmp(s, "delta") == 0) return SIMMY_DELTA;
  *ok = 0;
  return SIMMY_WYE;
}

static SimmyPmWaveform WaveformFromString(const char *s, int *ok) {
  *ok = 1;
  if (strcmp(s, "sine") == 0) return SIMMY_PM_SINE;
  if (strcmp(s, "trapezoid") == 0) return SIMMY_PM_TRAPEZOID;
  *ok = 0;
  return SIMMY_PM_SINE;
}

int SimmyBldcParamsFromJsonFile(const char *path, SimmyBldcParams *out,
                                 char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyBldcParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "bldcMotor");
  out->polePairs = (int)SimmyJsonNumber(SimmyJsonObjectGet(obj, "polePairs"), 0.0);

  const char *connection = SimmyJsonString(SimmyJsonObjectGet(obj, "connection"), "");
  if (connection[0]) {
    int ok;
    out->connection = ConnectionFromString(connection, &ok);
    if (!ok) {
      if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "bldcMotor.connection must be 'wye' or 'delta'");
      SimmyJsonFree(root);
      return 1;
    }
  }
  const char *waveform = SimmyJsonString(SimmyJsonObjectGet(obj, "waveform"), "");
  if (waveform[0]) {
    int ok;
    out->waveform = WaveformFromString(waveform, &ok);
    if (!ok) {
      if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "bldcMotor.waveform must be 'sine' or 'trapezoid'");
      SimmyJsonFree(root);
      return 1;
    }
  }

  FillDoubleArray(out->resistance, 3, SimmyJsonObjectGet(obj, "resistance"), 0.0);
  FillDoubleArray(out->inductance, 3, SimmyJsonObjectGet(obj, "inductance"), 0.0);
  FillDoubleArray(out->backEmfConstant, 3, SimmyJsonObjectGet(obj, "backEmfConstant"), 0.0);
  out->electricalOffset = SimmyJsonNumber(SimmyJsonObjectGet(obj, "electricalOffset"), 0.0);
  out->thirdHarmonic = SimmyJsonNumber(SimmyJsonObjectGet(obj, "thirdHarmonic"), 0.0);
  /* openWinding[3] is left at its zero/disabled default: a fault toggle
   * for the example sweeps, not a sample-motor parameter. */

  if (!SimmyBldcParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "bldcMotor parameters invalid: need polePairs > 0, nonnegative "
               "resistance/backEmfConstant per winding, and positive inductance per winding "
               "(inductance is required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyBridgeParamsFromJsonFile(const char *path, SimmyBridgeParams *out,
                                   char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyBridgeParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "bldcBridge");
  out->busVoltage = SimmyJsonNumber(SimmyJsonObjectGet(obj, "busVoltage"), 0.0);
  out->diodeDrop = SimmyJsonNumber(SimmyJsonObjectGet(obj, "diodeDrop"), 0.0);
  FillDoubleArray(out->onResistance, 6, SimmyJsonObjectGet(obj, "onResistance"), 0.0);
  /* openSwitch[6] is left at its zero/disabled default -- a fault toggle. */

  if (!SimmyBridgeParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "bldcBridge parameters invalid: need busVoltage > 0 "
               "(required, no default), nonnegative diodeDrop, and nonnegative onResistance "
               "per channel");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyPmParamsFromJsonFile(const char *path, SimmyPmParams *out,
                               char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyPmParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "pmMotor");
  out->polePairs = (int)SimmyJsonNumber(SimmyJsonObjectGet(obj, "polePairs"), 0.0);
  const char *waveform = SimmyJsonString(SimmyJsonObjectGet(obj, "waveform"), "");
  if (waveform[0]) {
    int ok;
    out->waveform = WaveformFromString(waveform, &ok);
    if (!ok) {
      if (errMsg && errMsgLen) snprintf(errMsg, errMsgLen, "pmMotor.waveform must be 'sine' or 'trapezoid'");
      SimmyJsonFree(root);
      return 1;
    }
  }
  out->resistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "resistance"), 0.0);
  out->inductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "inductance"), 0.0);
  out->backEmfConstant = SimmyJsonNumber(SimmyJsonObjectGet(obj, "backEmfConstant"), 0.0);
  out->electricalOffset = SimmyJsonNumber(SimmyJsonObjectGet(obj, "electricalOffset"), 0.0);

  if (!SimmyPmParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "pmMotor parameters invalid: need polePairs > 0, nonnegative "
               "resistance/backEmfConstant, and positive inductance (required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyDcParamsFromJsonFile(const char *path, SimmyDcParams *out,
                               char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyDcParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "dcMotor");
  out->resistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "resistance"), 0.0);
  out->inductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "inductance"), 0.0);
  out->backEmfConstant = SimmyJsonNumber(SimmyJsonObjectGet(obj, "backEmfConstant"), 0.0);

  if (!SimmyDcParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "dcMotor parameters invalid: need nonnegative resistance/"
               "backEmfConstant and positive inductance (required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyReluctanceParamsFromJsonFile(const char *path, SimmyReluctanceParams *out,
                                       char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyReluctanceParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "reluctanceMotor");
  out->resistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "resistance"), 0.0);
  out->minInductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "minInductance"), 0.0);
  out->maxInductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "maxInductance"), 0.0);
  out->shapeMeters = SimmyJsonNumber(SimmyJsonObjectGet(obj, "shapeMeters"), 0.0);
  out->saturationSharpness = SimmyJsonNumber(SimmyJsonObjectGet(obj, "saturationSharpness"), 0.0);
  out->minGapMeters = SimmyJsonNumber(SimmyJsonObjectGet(obj, "minGapMeters"), 0.0);

  if (!SimmyReluctanceParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "reluctanceMotor parameters invalid: need nonnegative "
               "resistance, positive minInductance, maxInductance > minInductance, and "
               "positive shapeMeters/saturationSharpness/minGapMeters (all required, no "
               "default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}

int SimmyVoiceCoilParamsFromJsonFile(const char *path, SimmyVoiceCoilParams *out,
                                      char *errMsg, size_t errMsgLen) {
  char *text = ReadWholeFile(path, errMsg, errMsgLen);
  if (!text) return 1;
  SimmyJsonValue *root = SimmyJsonParse(text, errMsg, errMsgLen);
  free(text);
  if (!root) return 1;

  SimmyVoiceCoilParams zero = {0};
  *out = zero;
  const SimmyJsonValue *obj = SimmyJsonObjectGet(root, "voiceCoilMotor");
  out->resistance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "resistance"), 0.0);
  out->inductance = SimmyJsonNumber(SimmyJsonObjectGet(obj, "inductance"), 0.0);
  out->forceConstant = SimmyJsonNumber(SimmyJsonObjectGet(obj, "forceConstant"), 0.0);

  if (!SimmyVoiceCoilParamsValid(out)) {
    if (errMsg && errMsgLen) {
      snprintf(errMsg, errMsgLen, "voiceCoilMotor parameters invalid: need nonnegative "
               "resistance/forceConstant and positive inductance (required, no default)");
    }
    SimmyJsonFree(root);
    return 1;
  }

  SimmyJsonFree(root);
  return 0;
}
