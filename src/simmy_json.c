#include "simmy_json.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct {
  const char *s;
  size_t pos, len;
  char *errMsg;
  size_t errMsgLen;
  int failed;
} Parser;

static void SetError(Parser *p, const char *msg) {
  if (p->failed) return;   /* keep the FIRST error, not a later cascade */
  if (p->errMsg && p->errMsgLen > 0) {
    snprintf(p->errMsg, p->errMsgLen, "%s at offset %zu", msg, p->pos);
  }
  p->failed = 1;
}

static void SkipWhitespace(Parser *p) {
  while (p->pos < p->len) {
    char c = p->s[p->pos];
    if (c != ' ' && c != '\t' && c != '\n' && c != '\r') break;
    p->pos++;
  }
}

static int Peek(Parser *p) { return p->pos < p->len ? (unsigned char)p->s[p->pos] : -1; }

static SimmyJsonValue *NewValue(SimmyJsonType type) {
  SimmyJsonValue *v = (SimmyJsonValue *)calloc(1, sizeof(SimmyJsonValue));
  v->type = type;
  return v;
}

static SimmyJsonValue *ParseValue(Parser *p);

static SimmyJsonValue *ParseString(Parser *p) {
  SimmyJsonValue *v = NewValue(SIMMY_JSON_STRING);
  p->pos++;   /* opening quote */
  char *buf = (char *)malloc(p->len - p->pos + 1);
  size_t bi = 0;
  while (p->pos < p->len && p->s[p->pos] != '"') {
    char c = p->s[p->pos];
    if (c == '\\' && p->pos + 1 < p->len) {
      p->pos++;
      switch (p->s[p->pos]) {
        case 'n': buf[bi++] = '\n'; break;
        case 't': buf[bi++] = '\t'; break;
        case 'r': buf[bi++] = '\r'; break;
        case '"': buf[bi++] = '"'; break;
        case '\\': buf[bi++] = '\\'; break;
        case '/': buf[bi++] = '/'; break;
        default: buf[bi++] = p->s[p->pos]; break;
      }
      p->pos++;
    } else {
      buf[bi++] = c;
      p->pos++;
    }
  }
  if (Peek(p) != '"') {
    SetError(p, "unterminated string");
    free(buf);
    v->as.stringValue = NULL;
    return v;
  }
  p->pos++;   /* closing quote */
  buf[bi] = '\0';
  v->as.stringValue = buf;
  return v;
}

static SimmyJsonValue *ParseNumber(Parser *p) {
  SimmyJsonValue *v = NewValue(SIMMY_JSON_NUMBER);
  size_t start = p->pos;
  if (Peek(p) == '-') p->pos++;
  while (p->pos < p->len && isdigit((unsigned char)p->s[p->pos])) p->pos++;
  if (Peek(p) == '.') {
    p->pos++;
    while (p->pos < p->len && isdigit((unsigned char)p->s[p->pos])) p->pos++;
  }
  if (Peek(p) == 'e' || Peek(p) == 'E') {
    p->pos++;
    if (Peek(p) == '+' || Peek(p) == '-') p->pos++;
    while (p->pos < p->len && isdigit((unsigned char)p->s[p->pos])) p->pos++;
  }
  char tmp[64];
  size_t n = p->pos - start;
  if (n >= sizeof(tmp)) n = sizeof(tmp) - 1;
  memcpy(tmp, p->s + start, n);
  tmp[n] = '\0';
  v->as.numberValue = atof(tmp);
  return v;
}

static int MatchLiteral(Parser *p, const char *lit) {
  size_t n = strlen(lit);
  if (p->pos + n > p->len || strncmp(p->s + p->pos, lit, n) != 0) return 0;
  p->pos += n;
  return 1;
}

static SimmyJsonValue *ParseArray(Parser *p) {
  SimmyJsonValue *v = NewValue(SIMMY_JSON_ARRAY);
  p->pos++;   /* '[' */
  SkipWhitespace(p);
  if (Peek(p) == ']') { p->pos++; return v; }
  for (;;) {
    SkipWhitespace(p);
    SimmyJsonValue *item = ParseValue(p);
    if (p->failed) { SimmyJsonFree(item); return v; }
    int idx = v->as.array.count++;
    v->as.array.items = (SimmyJsonValue **)realloc(v->as.array.items,
        (size_t)v->as.array.count * sizeof(SimmyJsonValue *));
    v->as.array.items[idx] = item;
    SkipWhitespace(p);
    if (Peek(p) == ',') { p->pos++; continue; }
    if (Peek(p) == ']') { p->pos++; break; }
    SetError(p, "expected ',' or ']' in array");
    break;
  }
  return v;
}

static SimmyJsonValue *ParseObject(Parser *p) {
  SimmyJsonValue *v = NewValue(SIMMY_JSON_OBJECT);
  p->pos++;   /* '{' */
  SkipWhitespace(p);
  if (Peek(p) == '}') { p->pos++; return v; }
  for (;;) {
    SkipWhitespace(p);
    if (Peek(p) != '"') { SetError(p, "expected string key in object"); break; }
    SimmyJsonValue *keyVal = ParseString(p);
    if (p->failed) { SimmyJsonFree(keyVal); break; }
    SkipWhitespace(p);
    if (Peek(p) != ':') { SetError(p, "expected ':' after object key"); SimmyJsonFree(keyVal); break; }
    p->pos++;
    SkipWhitespace(p);
    SimmyJsonValue *val = ParseValue(p);
    if (p->failed) { SimmyJsonFree(keyVal); SimmyJsonFree(val); break; }
    int idx = v->as.object.count++;
    v->as.object.keys = (char **)realloc(v->as.object.keys, (size_t)v->as.object.count * sizeof(char *));
    v->as.object.values = (SimmyJsonValue **)realloc(v->as.object.values,
        (size_t)v->as.object.count * sizeof(SimmyJsonValue *));
    v->as.object.keys[idx] = strdup(keyVal->as.stringValue ? keyVal->as.stringValue : "");
    v->as.object.values[idx] = val;
    SimmyJsonFree(keyVal);
    SkipWhitespace(p);
    if (Peek(p) == ',') { p->pos++; continue; }
    if (Peek(p) == '}') { p->pos++; break; }
    SetError(p, "expected ',' or '}' in object");
    break;
  }
  return v;
}

static SimmyJsonValue *ParseValue(Parser *p) {
  SkipWhitespace(p);
  int c = Peek(p);
  if (c == '{') return ParseObject(p);
  if (c == '[') return ParseArray(p);
  if (c == '"') return ParseString(p);
  if (c == '-' || (c >= '0' && c <= '9')) return ParseNumber(p);
  if (MatchLiteral(p, "true")) { SimmyJsonValue *v = NewValue(SIMMY_JSON_BOOL); v->as.boolValue = 1; return v; }
  if (MatchLiteral(p, "false")) { SimmyJsonValue *v = NewValue(SIMMY_JSON_BOOL); v->as.boolValue = 0; return v; }
  if (MatchLiteral(p, "null")) return NewValue(SIMMY_JSON_NULL);
  SetError(p, "unexpected character");
  return NewValue(SIMMY_JSON_NULL);
}

SimmyJsonValue *SimmyJsonParse(const char *text, char *errMsg, size_t errMsgLen) {
  Parser p;
  p.s = text;
  p.pos = 0;
  p.len = strlen(text);
  p.errMsg = errMsg;
  p.errMsgLen = errMsgLen;
  p.failed = 0;

  SimmyJsonValue *v = ParseValue(&p);
  SkipWhitespace(&p);
  if (!p.failed && p.pos != p.len) SetError(&p, "trailing data after JSON value");
  if (p.failed) {
    SimmyJsonFree(v);
    return NULL;
  }
  return v;
}

void SimmyJsonFree(SimmyJsonValue *v) {
  if (!v) return;
  switch (v->type) {
    case SIMMY_JSON_STRING:
      free(v->as.stringValue);
      break;
    case SIMMY_JSON_ARRAY:
      for (int i = 0; i < v->as.array.count; i++) SimmyJsonFree(v->as.array.items[i]);
      free(v->as.array.items);
      break;
    case SIMMY_JSON_OBJECT:
      for (int i = 0; i < v->as.object.count; i++) {
        free(v->as.object.keys[i]);
        SimmyJsonFree(v->as.object.values[i]);
      }
      free(v->as.object.keys);
      free(v->as.object.values);
      break;
    default:
      break;
  }
  free(v);
}

const SimmyJsonValue *SimmyJsonObjectGet(const SimmyJsonValue *obj, const char *key) {
  if (!obj || obj->type != SIMMY_JSON_OBJECT) return NULL;
  for (int i = 0; i < obj->as.object.count; i++)
    if (strcmp(obj->as.object.keys[i], key) == 0) return obj->as.object.values[i];
  return NULL;
}

int SimmyJsonArrayLength(const SimmyJsonValue *arr) {
  return (arr && arr->type == SIMMY_JSON_ARRAY) ? arr->as.array.count : 0;
}

const SimmyJsonValue *SimmyJsonArrayGet(const SimmyJsonValue *arr, int index) {
  if (!arr || arr->type != SIMMY_JSON_ARRAY || index < 0 || index >= arr->as.array.count) return NULL;
  return arr->as.array.items[index];
}

double SimmyJsonNumber(const SimmyJsonValue *v, double defaultValue) {
  return (v && v->type == SIMMY_JSON_NUMBER) ? v->as.numberValue : defaultValue;
}

const char *SimmyJsonString(const SimmyJsonValue *v, const char *defaultValue) {
  return (v && v->type == SIMMY_JSON_STRING && v->as.stringValue) ? v->as.stringValue : defaultValue;
}
