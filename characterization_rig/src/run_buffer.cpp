#include "run_buffer.h"
#include <string.h>

static command_buffer_t s_cmdbuf;
static capture_stream_t s_streams[NUM_CAPTURE_STREAMS];

command_buffer_t *cmdbuf_instance(void) { return &s_cmdbuf; }
capture_stream_t *capstream_instance(uint32_t stream_id) {
    return (stream_id < NUM_CAPTURE_STREAMS) ? &s_streams[stream_id] : 0;
}

/* ------------------------- command buffer ------------------------- */

void cmdbuf_reset(command_buffer_t *cb, uint32_t declared_count) {
    memset(cb->samples, 0, sizeof(cb->samples));
    memset(cb->filled, 0, sizeof(cb->filled));
    cb->declared_count = declared_count;
    cb->highest_filled_plus1 = 0;
}

static inline bool bit_get(const uint8_t *bits, uint32_t i) {
    return (bits[i >> 3] >> (i & 7)) & 1u;
}
static inline void bit_set(uint8_t *bits, uint32_t i) {
    bits[i >> 3] |= (uint8_t)(1u << (i & 7));
}

bool cmdbuf_put(command_buffer_t *cb, uint32_t offset, const int64_t *values, uint32_t count,
                const char **err_code) {
    if (count == 0) return true;
    if (offset > cb->declared_count || count > cb->declared_count - offset) {
        if (err_code) *err_code = "RANGE";
        return false;
    }
    if (offset + count > MAX_COMMAND_SAMPLES) {
        if (err_code) *err_code = "RANGE";
        return false;
    }
    for (uint32_t i = 0; i < count; i++) {
        int64_t v = values[i];
        if (v < INT32_MIN || v > INT32_MAX) {
            if (err_code) *err_code = "RANGE";
            return false;
        }
        uint32_t idx = offset + i;
        if (bit_get(cb->filled, idx) && cb->samples[idx] != (int32_t)v) {
            if (err_code) *err_code = "CONFLICT";
            return false;
        }
        cb->samples[idx] = (int32_t)v;
        bit_set(cb->filled, idx);
        if (idx + 1 > cb->highest_filled_plus1) cb->highest_filled_plus1 = idx + 1;
    }
    return true;
}

bool cmdbuf_is_complete(const command_buffer_t *cb) {
    if (cb->declared_count == 0) return false;
    for (uint32_t i = 0; i < cb->declared_count; i++) {
        if (!bit_get(cb->filled, i)) return false;
    }
    return true;
}

int32_t cmdbuf_value_at(const command_buffer_t *cb, uint32_t index) {
    return cb->samples[index];
}

void cmdbuf_hash_canonical(const command_buffer_t *cb, uint8_t out_digest[32]) {
    sha256_ctx ctx;
    sha256_init(&ctx);
    uint8_t le[8];
    for (uint32_t i = 0; i < cb->declared_count; i++) {
        int64_t v = cb->samples[i];
        uint64_t u = (uint64_t)v;
        for (int b = 0; b < 8; b++) le[b] = (uint8_t)(u >> (8 * b));
        sha256_update(&ctx, le, 8);
    }
    sha256_final(&ctx, out_digest);
}

/* ------------------------- capture streams ------------------------- */

void capstream_reset(capture_stream_t *cs, uint32_t capacity, uint64_t period_ticks, uint8_t channels) {
    uint32_t max_cap = MAX_CAPTURE_SAMPLES / (channels ? channels : 1);
    if (capacity > max_cap) capacity = max_cap;
    memset(cs->samples, 0, sizeof(cs->samples));
    cs->count = 0;
    cs->capacity = capacity;
    cs->first_sample_tick = 0;
    cs->period_ticks = period_ticks;
    cs->channels = channels;
    cs->overflowed = false;
}

bool capstream_append(capture_stream_t *cs, const int32_t *values) {
    if (cs->count >= cs->capacity) {
        cs->overflowed = true;
        return false;
    }
    uint32_t base = cs->count * cs->channels;
    for (uint8_t c = 0; c < cs->channels; c++) cs->samples[base + c] = values[c];
    cs->count++;
    return true;
}

void capstream_hash(const capture_stream_t *cs, bool widen_to_i64, uint8_t out_digest[32]) {
    sha256_ctx ctx;
    sha256_init(&ctx);
    uint32_t total = cs->count * cs->channels;
    if (widen_to_i64) {
        uint8_t le[8];
        for (uint32_t i = 0; i < total; i++) {
            uint64_t u = (uint64_t)(int64_t)cs->samples[i];
            for (int b = 0; b < 8; b++) le[b] = (uint8_t)(u >> (8 * b));
            sha256_update(&ctx, le, 8);
        }
    } else {
        uint8_t le[4];
        for (uint32_t i = 0; i < total; i++) {
            uint32_t u = (uint32_t)cs->samples[i];
            for (int b = 0; b < 4; b++) le[b] = (uint8_t)(u >> (8 * b));
            sha256_update(&ctx, le, 4);
        }
    }
    sha256_final(&ctx, out_digest);
}

static inline void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void put_u64le(uint8_t *p, uint64_t v) {
    for (int b = 0; b < 8; b++) p[b] = (uint8_t)(v >> (8 * b));
}
static inline void put_i32le(uint8_t *p, int32_t v) { put_u32le(p, (uint32_t)v); }
static inline void put_i64le(uint8_t *p, int64_t v) { put_u64le(p, (uint64_t)v); }

uint32_t capstream_encode_block(const capture_stream_t *cs, uint32_t block_index,
                                 uint32_t block_len, uint8_t codec,
                                 uint8_t *out, uint32_t out_cap) {
    if (block_len == 0) return 0;
    uint32_t start = block_index * block_len; /* caller picks a fixed block_len and derives block_index/start */
    if (start >= cs->count) return 0;
    uint32_t avail = cs->count - start;
    uint32_t n = (avail < block_len) ? avail : block_len;
    uint8_t ch = cs->channels;

    if (codec == 1) { /* raw_i64 */
        uint32_t need = n * ch * 8;
        if (need > out_cap) return 0;
        for (uint32_t i = 0; i < n * ch; i++) {
            put_i64le(out + i * 8, (int64_t)cs->samples[start * ch + i]);
        }
        return need;
    }
    if (codec == 3) { /* raw_i32 */
        uint32_t need = n * ch * 4;
        if (need > out_cap) return 0;
        for (uint32_t i = 0; i < n * ch; i++) {
            put_i32le(out + i * 4, cs->samples[start * ch + i]);
        }
        return need;
    }
    if (codec == 2) { /* delta_rle_i32, single-channel streams only in this build */
        if (ch != 1) return 0;
        uint32_t off = 0;
        if (8 > out_cap) return 0;
        put_i64le(out, (int64_t)cs->samples[start]);
        off += 8;
        uint32_t i = 1;
        while (i < n) {
            int32_t delta = cs->samples[start + i] - cs->samples[start + i - 1];
            uint32_t repeat = 1;
            while (i + repeat < n &&
                   (cs->samples[start + i + repeat] - cs->samples[start + i + repeat - 1]) == delta) {
                repeat++;
            }
            if (off + 4 + 4 > out_cap) return 0;
            put_u32le(out + off, repeat); off += 4;
            put_i32le(out + off, delta); off += 4;
            i += repeat;
        }
        return off;
    }
    return 0;
}
