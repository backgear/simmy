#ifndef SIMMY_JSON_H
#define SIMMY_JSON_H

#include <stddef.h>

/*
 * A small JSON value parser, scoped to exactly what simmy's own config
 * files need: objects, arrays, numbers, strings, true/false/null. NOT a
 * general-purpose, hardened JSON parser -- no unicode escapes beyond the
 * common \n/\t/\r/\"/\\/\/ , no attempt at handling malicious/adversarial
 * input gracefully. Hand-rolled rather than vendored, same as every other
 * primitive in this library.
 */

typedef enum {
  SIMMY_JSON_NULL,
  SIMMY_JSON_BOOL,
  SIMMY_JSON_NUMBER,
  SIMMY_JSON_STRING,
  SIMMY_JSON_ARRAY,
  SIMMY_JSON_OBJECT,
} SimmyJsonType;

typedef struct SimmyJsonValue SimmyJsonValue;

struct SimmyJsonValue {
  SimmyJsonType type;
  union {
    int boolValue;
    double numberValue;
    char *stringValue;
    struct {
      SimmyJsonValue **items;
      int count;
    } array;
    struct {
      char **keys;
      SimmyJsonValue **values;
      int count;
    } object;
  } as;
};

/* Parses `text` (NUL-terminated). Returns the root value on success (the
 * caller must SimmyJsonFree it), or NULL on failure (errMsg filled, if
 * non-NULL, with a human-readable message truncated to errMsgLen). */
SimmyJsonValue *SimmyJsonParse(const char *text, char *errMsg, size_t errMsgLen);
void SimmyJsonFree(SimmyJsonValue *value);

/* Lenient accessors: return NULL/0/the given default if the value isn't
 * the expected shape or the key/index doesn't exist, rather than erroring --
 * config fields are optional throughout, see simmy_config.c. */
const SimmyJsonValue *SimmyJsonObjectGet(const SimmyJsonValue *obj, const char *key);
int SimmyJsonArrayLength(const SimmyJsonValue *arr);
const SimmyJsonValue *SimmyJsonArrayGet(const SimmyJsonValue *arr, int index);
double SimmyJsonNumber(const SimmyJsonValue *value, double defaultValue);
const char *SimmyJsonString(const SimmyJsonValue *value, const char *defaultValue);

#endif /* SIMMY_JSON_H */
