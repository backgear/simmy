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

void capstream_reset(capture_stream_t *cs, uint64_t period_ticks, uint8_t channels,
                      bool widen_hash_to_i64) {
    memset(cs->staging, 0, sizeof(cs->staging));
    cs->staging_fill[0] = cs->staging_fill[1] = 0;
    cs->chunk_first_tick[0] = cs->chunk_first_tick[1] = 0;
    cs->active = 0;
    cs->pending[0] = cs->pending[1] = false;
    cs->next_push_seq = 0;
    cs->count = 0;
    cs->first_sample_tick = 0;
    cs->period_ticks = period_ticks;
    cs->channels = channels;
    cs->widen_hash_to_i64 = widen_hash_to_i64;
    cs->overflowed = false;
    sha256_init(&cs->hash_ctx);
}

bool capstream_append(capture_stream_t *cs, const int32_t *values) {
    if (cs->overflowed) return false;
    uint8_t half = cs->active;
    if (cs->staging_fill[half] >= PUSH_CHUNK_SAMPLES) {
        cs->overflowed = true;
        return false;
    }
    if (cs->staging_fill[half] == 0) {
        cs->chunk_first_tick[half] = cs->first_sample_tick + (uint64_t)cs->count * cs->period_ticks;
    }
    uint32_t base = cs->staging_fill[half] * cs->channels;
    for (uint8_t c = 0; c < cs->channels; c++) cs->staging[half][base + c] = values[c];
    cs->staging_fill[half]++;
    cs->count++;

    if (cs->staging_fill[half] == PUSH_CHUNK_SAMPLES) {
        uint8_t other = (uint8_t)(1 - half);
        if (cs->pending[other]) {
            /* The main loop hasn't drained the other half yet -- the host
             * (or USB) isn't keeping up. This is the real overflow now. */
            cs->overflowed = true;
            return false;
        }
        cs->pending[half] = true;
        cs->active = other;
        cs->staging_fill[other] = 0;
    }
    return true;
}

void capstream_flush(capture_stream_t *cs) {
    uint8_t half = cs->active;
    if (cs->staging_fill[half] > 0 && !cs->pending[half]) cs->pending[half] = true;
}

static inline void put_u32le(uint8_t *p, uint32_t v) {
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}
static inline void put_i32le(uint8_t *p, int32_t v) { put_u32le(p, (uint32_t)v); }
static inline void put_i64le(uint8_t *p, int64_t v) {
    uint64_t u = (uint64_t)v;
    for (int b = 0; b < 8; b++) p[b] = (uint8_t)(u >> (8 * b));
}

/* Encodes staging[half][0 .. n*channels) with the given codec, exactly
 * the same wire shapes the old whole-buffer encoder produced. */
static uint32_t encode_chunk(const int32_t *data, uint32_t n, uint8_t channels, uint8_t codec,
                              uint8_t *out, uint32_t out_cap) {
    if (codec == 1) { /* raw_i64 */
        uint32_t need = n * channels * 8;
        if (need > out_cap) return 0;
        for (uint32_t i = 0; i < n * channels; i++) put_i64le(out + i * 8, (int64_t)data[i]);
        return need;
    }
    if (codec == 3) { /* raw_i32 */
        uint32_t need = n * channels * 4;
        if (need > out_cap) return 0;
        for (uint32_t i = 0; i < n * channels; i++) put_i32le(out + i * 4, data[i]);
        return need;
    }
    if (codec == 2) { /* delta_rle_i32, single-channel streams only in this build */
        if (channels != 1 || n == 0) return 0;
        uint32_t off = 0;
        if (8 > out_cap) return 0;
        put_i64le(out, (int64_t)data[0]);
        off += 8;
        uint32_t i = 1;
        while (i < n) {
            int32_t delta = data[i] - data[i - 1];
            uint32_t repeat = 1;
            while (i + repeat < n && (data[i + repeat] - data[i + repeat - 1]) == delta) repeat++;
            if (off + 4 + 4 > out_cap) return 0;
            put_u32le(out + off, repeat); off += 4;
            put_i32le(out + off, delta); off += 4;
            i += repeat;
        }
        return off;
    }
    return 0;
}

bool capstream_drain(capture_stream_t *cs, uint8_t codec, uint8_t *body, uint32_t body_cap,
                      uint32_t *out_sample_count, uint64_t *out_first_tick,
                      uint32_t *out_seq, uint32_t *out_body_len) {
    /* Flush can leave both halves pending. Half index is not chronological:
     * the final short half 0 can follow a full, older half 1. */
    bool pending0 = cs->pending[0], pending1 = cs->pending[1];
    if (!pending0 && !pending1) return false;
    uint8_t half = !pending0 ? 1 : !pending1 ? 0 :
        (cs->chunk_first_tick[1] < cs->chunk_first_tick[0] ? 1 : 0);
    {
        uint32_t n = cs->staging_fill[half];
        const int32_t *data = cs->staging[half];

        /* Hash commits to the uncompressed raw values regardless of which
         * wire codec this chunk happens to use. */
        uint32_t total = n * cs->channels;
        uint8_t le[8];
        if (cs->widen_hash_to_i64) {
            for (uint32_t i = 0; i < total; i++) {
                uint64_t u = (uint64_t)(int64_t)data[i];
                for (int b = 0; b < 8; b++) le[b] = (uint8_t)(u >> (8 * b));
                sha256_update(&cs->hash_ctx, le, 8);
            }
        } else {
            for (uint32_t i = 0; i < total; i++) {
                uint32_t u = (uint32_t)data[i];
                for (int b = 0; b < 4; b++) le[b] = (uint8_t)(u >> (8 * b));
                sha256_update(&cs->hash_ctx, le, 4);
            }
        }

        uint32_t encoded = encode_chunk(data, n, cs->channels, codec, body, body_cap);
        if (encoded == 0 && n > 0) return false; /* caller's body_cap was sized wrong; don't lose the chunk */

        *out_sample_count = n;
        *out_first_tick = cs->chunk_first_tick[half];
        *out_seq = cs->next_push_seq++;
        *out_body_len = encoded;
        cs->staging_fill[half] = 0;
        cs->pending[half] = false;
        return true;
    }
}

void capstream_final_hash(const capture_stream_t *cs, uint8_t out_digest[32]) {
    sha256_ctx copy = cs->hash_ctx; /* plain POD struct: finalize a copy, keep the live one running */
    sha256_final(&copy, out_digest);
}
