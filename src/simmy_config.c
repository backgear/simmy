#include "simmy.h"
#include "simmy_json.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* See README.md for the full config file schema. */

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
