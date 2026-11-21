#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Minimal append-only JSON builder into a fixed caller-owned buffer.
 * Used for every outgoing RESPONSE/EVENT payload so response shape is
 * fully controlled here rather than round-tripped through a generic
 * JSON library (which is instead used only to parse incoming requests).
 * Not reentrant / not for nested documents beyond JW_MAX_DEPTH. */
#define JW_MAX_DEPTH 8

typedef struct {
    char *buf;
    size_t cap;
    size_t len;
    int depth;
    bool need_comma[JW_MAX_DEPTH];
    bool overflow;
} json_writer_t;

void jw_init(json_writer_t *w, char *buf, size_t cap);
void jw_obj_begin(json_writer_t *w);
void jw_obj_end(json_writer_t *w);
void jw_arr_begin(json_writer_t *w);
void jw_arr_end(json_writer_t *w);
void jw_key(json_writer_t *w, const char *key);     /* call before a value in an object */
void jw_str(json_writer_t *w, const char *s);        /* JSON-escaped string value */
void jw_raw(json_writer_t *w, const char *literal);  /* unescaped literal: number/bool/null/pre-built JSON */
void jw_i64(json_writer_t *w, int64_t v);
void jw_u64(json_writer_t *w, uint64_t v);
void jw_bool(json_writer_t *w, bool v);
void jw_null(json_writer_t *w);
/* Convenience: key+value in one call. */
void jw_kv_str(json_writer_t *w, const char *key, const char *val);
void jw_kv_i64(json_writer_t *w, const char *key, int64_t val);
void jw_kv_u64(json_writer_t *w, const char *key, uint64_t val);
void jw_kv_bool(json_writer_t *w, const char *key, bool val);
void jw_kv_null(json_writer_t *w, const char *key);

/* Finalizes and returns the written length, or 0 on overflow. */
size_t jw_finish(json_writer_t *w);

#ifdef __cplusplus
}
#endif
