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
 * Capture streams: no full-run buffer at all -- samples are pushed off
 * the device continuously, unsolicited, as soon as a small chunk is
 * ready, reusing the USB CDC stack's own small TX queue as the only
 * elastic pipe (see protocol.cpp's capture_streams_service). RAM here is
 * a fixed, tiny double buffer (PUSH_CHUNK_SAMPLES samples x 2 halves),
 * completely independent of how long the run lasts -- a 2-second 100kHz
 * capture and a 2-minute one cost exactly the same SRAM.
 *
 * The ISR (capstream_append) only ever touches the "active" half, which
 * flips to the other half once full; capstream_drain (called from the
 * main loop, never the ISR) encodes and clears whichever half is
 * pending. If the main loop hasn't drained the previous half by the
 * time the ISR fills the next one, that's the real overflow condition
 * now: the host (or USB) isn't keeping up, not "ran out of buffer."
 * ------------------------------------------------------------------- */
typedef struct {
    int32_t staging[2][PUSH_CHUNK_SAMPLES * NUM_ANALOG_CHANNELS];
    uint32_t staging_fill[2];       /* samples currently held in each half */
    uint64_t chunk_first_tick[2];   /* device tick of each half's first sample */
    volatile uint8_t active;        /* which half the ISR is currently writing into */
    volatile bool pending[2];       /* half is full (or flushed) and awaiting drain */
    uint32_t next_push_seq;         /* this stream's own monotonic push-frame counter */
    volatile uint32_t count;        /* total logical samples captured so far, this run */
    uint64_t first_sample_tick;     /* absolute device tick of the run's sample 0 */
    uint64_t period_ticks;
    uint8_t channels;                /* 1 for encoder streams, NUM_ANALOG_CHANNELS for analog */
    bool widen_hash_to_i64;          /* matches the codec width GET_RUN's sha256 commits to */
    bool overflowed;                 /* host/USB fell behind -> run faults, capture incomplete */
    sha256_ctx hash_ctx;              /* incremental whole-stream hash, fed per drained chunk */
} capture_stream_t;

void capstream_reset(capture_stream_t *cs, uint64_t period_ticks, uint8_t channels,
                      bool widen_hash_to_i64);
/* Appends one sample (encoder: 1 int32 count) or one frame of `channels`
 * interleaved int32 values (analog) into the active staging half. Called
 * from ISR context only. Returns false (and sets overflowed) if the
 * other half is still pending -- the main loop hasn't drained it yet. */
bool capstream_append(capture_stream_t *cs, const int32_t *values);

/* Marks whatever's in the active half as pending even if not full yet --
 * call once, from non-ISR context, right when a run reaches its terminal
 * state, so the final partial chunk still gets pushed. Idempotent. */
void capstream_flush(capture_stream_t *cs);

/* If a staging half is pending, feeds it into this stream's running
 * whole-capture hash, encodes it with the given codec (1=raw_i64,
 * 2=delta_rle_i32, 3=raw_i32) into `body` (caller sizes `body_cap` from
 * PUSH_CHUNK_SAMPLES/AXIS_MAX_PAYLOAD), frees that half, and reports its
 * metadata. Returns false (leaving all outputs untouched) if nothing was
 * pending. Call from the main loop only, never the ISR. */
bool capstream_drain(capture_stream_t *cs, uint8_t codec, uint8_t *body, uint32_t body_cap,
                      uint32_t *out_sample_count, uint64_t *out_first_tick,
                      uint32_t *out_seq, uint32_t *out_body_len);

/* Finalizes a *copy* of this stream's running hash (leaves it live, so
 * repeated GET_RUN calls are idempotent) -- the whole-stream SHA-256 over
 * concatenated uncompressed sample-major little-endian raw values, at
 * the element width `widen_hash_to_i64` declared at capstream_reset. */
void capstream_final_hash(const capture_stream_t *cs, uint8_t out_digest[32]);

/* Single static instance of each buffer, reused across runs (CLEAR/BEGIN
 * reset them). One place to own this storage so both protocol.cpp
 * (uploads, GET_RUN, capture push) and axis_hw.cpp (ISR-driven capture
 * append, command sample readback while RUNNING) share the same memory
 * instead of copying it. */
command_buffer_t *cmdbuf_instance(void);
capture_stream_t *capstream_instance(uint32_t stream_id); /* stream_id < NUM_CAPTURE_STREAMS */

#ifdef __cplusplus
}
#endif
