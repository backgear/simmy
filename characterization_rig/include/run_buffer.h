#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "axis_config.h"
#include "sha256.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------------
 * Command buffer: the uploaded plan (PUT_COMMAND), tracked for partial
 * uploads via a completeness bitmap as required ("no partially uploaded
 * plan can execute"). Stored internally as int32_t -- this firmware's
 * declared command-sample range limit, even though the wire/SEAL hash
 * format is signed i64 (cmdbuf_hash_canonical widens on the fly so no
 * i64 array is ever materialized).
 * ------------------------------------------------------------------- */
typedef struct {
    int32_t samples[MAX_COMMAND_SAMPLES];
    uint8_t filled[(MAX_COMMAND_SAMPLES + 7) / 8];
    uint32_t declared_count;   /* from BEGIN's manifest; 0 until set */
    uint32_t highest_filled_plus1;
} command_buffer_t;

void cmdbuf_reset(command_buffer_t *cb, uint32_t declared_count);
/* Returns false on RANGE (offset/count out of bounds) or CONFLICT
 * (value disagrees with an already-written sample at that offset). */
bool cmdbuf_put(command_buffer_t *cb, uint32_t offset, const int64_t *values, uint32_t count,
                const char **err_code);
bool cmdbuf_is_complete(const command_buffer_t *cb);
/* Streams the canonical i64-little-endian representation through sha256,
 * widening each stored int32_t sample on the fly. */
void cmdbuf_hash_canonical(const command_buffer_t *cb, uint8_t out_digest[32]);
int32_t cmdbuf_value_at(const command_buffer_t *cb, uint32_t index);

/* ---------------------------------------------------------------------
 * Capture streams: one ring-free (run is finite, so just linear) buffer
 * per stream, appended to from the service ISR while RUNNING.
 * ------------------------------------------------------------------- */
typedef struct {
    int32_t samples[MAX_CAPTURE_SAMPLES];
    volatile uint32_t count;      /* samples actually captured so far */
    uint32_t capacity;            /* <= MAX_CAPTURE_SAMPLES, set at BEGIN from manifest */
    uint64_t first_sample_tick;   /* absolute device tick of sample 0 */
    uint64_t period_ticks;
    uint8_t channels;             /* 1 for encoder streams, NUM_ANALOG_CHANNELS for analog */
    bool overflowed;              /* capacity exceeded -> run faults, capture incomplete */
} capture_stream_t;

void capstream_reset(capture_stream_t *cs, uint32_t capacity, uint64_t period_ticks, uint8_t channels);
/* Appends one sample (encoder: 1 int32 count) or one frame of `channels`
 * interleaved int32 values (analog). Called from ISR context only.
 * Returns false (and sets overflowed) if capacity was exceeded. */
bool capstream_append(capture_stream_t *cs, const int32_t *values);

/* Whole-stream SHA-256 over concatenated uncompressed sample-major
 * little-endian raw values, at the declared element width (8 bytes for
 * encoder/raw_i64 streams, 4 bytes for the raw_i32 analog stream). */
void capstream_hash(const capture_stream_t *cs, bool widen_to_i64, uint8_t out_digest[32]);

/* Encodes one block's worth of samples ([start, start+block_len)) into
 * `out` using the given codec (1=raw_i64, 2=delta_rle_i32, 3=raw_i32).
 * Returns the number of bytes written, or 0 if codec/range is invalid.
 * `out_cap` bounds the write (caller sizes it from AXIS_MAX_PAYLOAD). */
uint32_t capstream_encode_block(const capture_stream_t *cs, uint32_t block_index,
                                 uint32_t block_len, uint8_t codec,
                                 uint8_t *out, uint32_t out_cap);

/* Single static instance of each buffer, reused across runs (CLEAR/BEGIN
 * reset them). One place to own this storage so both protocol.cpp
 * (uploads, GET_RUN/GET_BLOCK) and axis_hw.cpp (ISR-driven capture
 * append, command sample readback while RUNNING) share the same memory
 * instead of copying it. */
command_buffer_t *cmdbuf_instance(void);
capture_stream_t *capstream_instance(uint32_t stream_id); /* stream_id < NUM_CAPTURE_STREAMS */

#ifdef __cplusplus
}
#endif
