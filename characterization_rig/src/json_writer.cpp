#include "json_writer.h"
#include <stdio.h>
#include <string.h>

/* Design: each writer tracks, per nesting depth, whether a sibling
 * element/member has already been written at that depth (need_comma).
 * `awaiting_value` is true immediately after jw_key() so the very next
 * value-writing call knows it is filling that key's value (no comma,
 * no sibling bookkeeping of its own) rather than being a fresh array
 * element or object member. */

/* File-local rather than in json_writer_t: this firmware only ever
 * builds one response at a time on the main thread, so a single flag
 * reset in jw_init is simpler than threading it through every call. */
static bool s_awaiting_value = false;

void jw_init(json_writer_t *w, char *buf, size_t cap) {
    w->buf = buf;
    w->cap = cap;
    w->len = 0;
    w->depth = 0;
    w->overflow = false;
    w->sink = nullptr;
    w->sink_ctx = nullptr;
    for (int i = 0; i < JW_MAX_DEPTH; i++) w->need_comma[i] = false;
    s_awaiting_value = false;
}

void jw_init_sink(json_writer_t *w,
                  void (*sink)(void *, const char *, size_t), void *ctx) {
    jw_init(w, nullptr, 0);
    w->sink = sink;
    w->sink_ctx = ctx;
}

static void put(json_writer_t *w, const char *s, size_t n) {
    if (w->overflow) return;
    if (w->sink) {
        w->sink(w->sink_ctx, s, n);
        w->len += n;
        return;
    }
    if (w->len + n >= w->cap) { w->overflow = true; return; }
    memcpy(w->buf + w->len, s, n);
    w->len += n;
}
static void putc1(json_writer_t *w, char c) { put(w, &c, 1); }

static void begin_element(json_writer_t *w) {
    if (s_awaiting_value) {
        s_awaiting_value = false;
        return; /* filling a key's value: no comma, no sibling marking */
    }
    if (w->depth > 0) {
        if (w->need_comma[w->depth - 1]) putc1(w, ',');
        w->need_comma[w->depth - 1] = true;
    }
}

void jw_obj_begin(json_writer_t *w) {
    begin_element(w);
    putc1(w, '{');
    if (w->depth < JW_MAX_DEPTH) w->need_comma[w->depth] = false;
    w->depth++;
}
void jw_obj_end(json_writer_t *w) {
    if (w->depth > 0) w->depth--;
    putc1(w, '}');
}
void jw_arr_begin(json_writer_t *w) {
    begin_element(w);
    putc1(w, '[');
    if (w->depth < JW_MAX_DEPTH) w->need_comma[w->depth] = false;
    w->depth++;
}
void jw_arr_end(json_writer_t *w) {
    if (w->depth > 0) w->depth--;
    putc1(w, ']');
}

void jw_key(json_writer_t *w, const char *key) {
    begin_element(w);
    putc1(w, '"');
    put(w, key, strlen(key));
    put(w, "\":", 2);
    s_awaiting_value = true;
}

static void str_escaped(json_writer_t *w, const char *s) {
    putc1(w, '"');
    for (const unsigned char *p = (const unsigned char *)s; *p; p++) {
        unsigned char c = *p;
        switch (c) {
            case '"': put(w, "\\\"", 2); break;
            case '\\': put(w, "\\\\", 2); break;
            case '\n': put(w, "\\n", 2); break;
            case '\r': put(w, "\\r", 2); break;
            case '\t': put(w, "\\t", 2); break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    int n = snprintf(buf, sizeof(buf), "\\u%04x", c);
                    put(w, buf, (size_t)n);
                } else {
                    putc1(w, (char)c);
                }
        }
    }
    putc1(w, '"');
}

void jw_str(json_writer_t *w, const char *s) {
    begin_element(w);
    str_escaped(w, s);
}

void jw_raw(json_writer_t *w, const char *literal) {
    begin_element(w);
    put(w, literal, strlen(literal));
}

/* Embedded printf implementations (notably newlib-nano) may omit the
 * long-long formats. Convert directly so every target emits JSON numbers. */
static void decimal(json_writer_t *w, uint64_t magnitude, bool negative) {
    char buf[21]; /* 20 decimal digits plus an optional minus sign */
    char *end = buf + sizeof(buf);
    char *p = end;
    do {
        *--p = (char)('0' + magnitude % 10);
        magnitude /= 10;
    } while (magnitude != 0);
    if (negative) *--p = '-';
    begin_element(w);
    put(w, p, (size_t)(end - p));
}

void jw_i64(json_writer_t *w, int64_t v) {
    /* Unsigned subtraction also handles INT64_MIN without signed overflow. */
    uint64_t magnitude = v < 0 ? UINT64_C(0) - (uint64_t)v : (uint64_t)v;
    decimal(w, magnitude, v < 0);
}
void jw_u64(json_writer_t *w, uint64_t v) {
    decimal(w, v, false);
}
void jw_bool(json_writer_t *w, bool v) { jw_raw(w, v ? "true" : "false"); }
void jw_null(json_writer_t *w) { jw_raw(w, "null"); }

void jw_kv_str(json_writer_t *w, const char *key, const char *val) { jw_key(w, key); jw_str(w, val); }
void jw_kv_i64(json_writer_t *w, const char *key, int64_t val) { jw_key(w, key); jw_i64(w, val); }
void jw_kv_u64(json_writer_t *w, const char *key, uint64_t val) { jw_key(w, key); jw_u64(w, val); }
void jw_kv_bool(json_writer_t *w, const char *key, bool val) { jw_key(w, key); jw_bool(w, val); }
void jw_kv_null(json_writer_t *w, const char *key) { jw_key(w, key); jw_null(w); }

size_t jw_finish(json_writer_t *w) {
    s_awaiting_value = false;
    return w->overflow ? 0 : w->len;
}
