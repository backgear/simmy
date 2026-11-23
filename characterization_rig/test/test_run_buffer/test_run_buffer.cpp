#include <unity.h>
#include "run_buffer.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* ------------------------- command buffer ------------------------- */

static void test_cmdbuf_put_and_complete(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 4);
    TEST_ASSERT_FALSE(cmdbuf_is_complete(&cb));

    const char *err = NULL;
    int64_t vals[2] = {10, 20};
    TEST_ASSERT_TRUE(cmdbuf_put(&cb, 0, vals, 2, &err));
    TEST_ASSERT_FALSE(cmdbuf_is_complete(&cb));

    int64_t vals2[2] = {30, 40};
    TEST_ASSERT_TRUE(cmdbuf_put(&cb, 2, vals2, 2, &err));
    TEST_ASSERT_TRUE(cmdbuf_is_complete(&cb));

    TEST_ASSERT_EQUAL_INT32(10, cmdbuf_value_at(&cb, 0));
    TEST_ASSERT_EQUAL_INT32(40, cmdbuf_value_at(&cb, 3));
}

static void test_cmdbuf_rejects_out_of_declared_range(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 4);
    const char *err = NULL;
    int64_t vals[1] = {1};
    TEST_ASSERT_FALSE(cmdbuf_put(&cb, 4, vals, 1, &err)); /* offset == declared_count */
    TEST_ASSERT_EQUAL_STRING("RANGE", err);

    err = NULL;
    int64_t vals2[2] = {1, 2};
    TEST_ASSERT_FALSE(cmdbuf_put(&cb, 3, vals2, 2, &err)); /* spills past declared_count */
    TEST_ASSERT_EQUAL_STRING("RANGE", err);
}

static void test_cmdbuf_rejects_value_out_of_i32_range(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 2);
    const char *err = NULL;
    int64_t vals[1] = {(int64_t)INT32_MAX + 1};
    TEST_ASSERT_FALSE(cmdbuf_put(&cb, 0, vals, 1, &err));
    TEST_ASSERT_EQUAL_STRING("RANGE", err);
}

static void test_cmdbuf_rewrite_same_value_is_ok_conflict_is_rejected(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 2);
    const char *err = NULL;
    int64_t vals[1] = {5};
    TEST_ASSERT_TRUE(cmdbuf_put(&cb, 0, vals, 1, &err));

    /* Re-sending the identical value at the same offset is idempotent. */
    TEST_ASSERT_TRUE(cmdbuf_put(&cb, 0, vals, 1, &err));

    /* A different value at an already-filled offset is a real conflict. */
    int64_t other[1] = {6};
    TEST_ASSERT_FALSE(cmdbuf_put(&cb, 0, other, 1, &err));
    TEST_ASSERT_EQUAL_STRING("CONFLICT", err);
}

static void test_cmdbuf_zero_declared_count_never_complete(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 0);
    TEST_ASSERT_FALSE(cmdbuf_is_complete(&cb));
}

static void test_cmdbuf_hash_canonical_matches_direct_sha256(void) {
    command_buffer_t cb;
    cmdbuf_reset(&cb, 3);
    const char *err = NULL;
    int64_t vals[3] = {1, -2, 3};
    TEST_ASSERT_TRUE(cmdbuf_put(&cb, 0, vals, 3, &err));

    uint8_t digest[32];
    cmdbuf_hash_canonical(&cb, digest);

    uint8_t le[8 * 3];
    for (int i = 0; i < 3; i++) {
        uint64_t u = (uint64_t)vals[i];
        for (int b = 0; b < 8; b++) le[i * 8 + b] = (uint8_t)(u >> (8 * b));
    }
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, le, sizeof(le));
    uint8_t expected[32];
    sha256_final(&ctx, expected);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, digest, 32);
}

/* ------------------------- capture streams: push model ------------------------- */

/* Drains at most one pending chunk; fails the test if nothing was pending,
 * since every call site below expects exactly one chunk to be ready. */
static uint32_t drain_one(capture_stream_t *cs, uint8_t codec, uint8_t *body, uint32_t body_cap,
                           uint64_t *first_tick, uint32_t *seq) {
    uint32_t n, blen;
    TEST_ASSERT_TRUE(capstream_drain(cs, codec, body, body_cap, &n, first_tick, seq, &blen));
    (void)blen;
    return n;
}

static void test_capstream_append_counts_and_chunks_at_push_size(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 100, 1, true);
    for (int32_t v = 0; v < (int32_t)PUSH_CHUNK_SAMPLES - 1; v++) {
        TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    }
    TEST_ASSERT_EQUAL_UINT32(PUSH_CHUNK_SAMPLES - 1, cs.count);
    TEST_ASSERT_FALSE(cs.pending[0]);
    TEST_ASSERT_FALSE(cs.pending[1]);

    int32_t last = (int32_t)PUSH_CHUNK_SAMPLES - 1;
    TEST_ASSERT_TRUE(capstream_append(&cs, &last)); /* fills the first half exactly */
    TEST_ASSERT_EQUAL_UINT32(PUSH_CHUNK_SAMPLES, cs.count);
    TEST_ASSERT_TRUE(cs.pending[0]);
    TEST_ASSERT_EQUAL_UINT8(1, cs.active); /* flipped to the other half */
    TEST_ASSERT_FALSE(cs.overflowed);
}

static void test_capstream_drain_reports_chunk_and_frees_half(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 10, 1, true);
    for (int32_t v = 0; v < (int32_t)PUSH_CHUNK_SAMPLES; v++) capstream_append(&cs, &v);
    TEST_ASSERT_TRUE(cs.pending[0]);

    uint8_t body[PUSH_CHUNK_SAMPLES * 8];
    uint64_t first_tick; uint32_t seq;
    uint32_t n = drain_one(&cs, 1 /* raw_i64 */, body, sizeof(body), &first_tick, &seq);
    TEST_ASSERT_EQUAL_UINT32(PUSH_CHUNK_SAMPLES, n);
    TEST_ASSERT_EQUAL_UINT64(0, first_tick); /* first_sample_tick=0 at reset, period irrelevant for chunk 0 */
    TEST_ASSERT_EQUAL_UINT32(0, seq); /* first pushed chunk for this stream */
    TEST_ASSERT_FALSE(cs.pending[0]); /* drained -> freed */

    int64_t v0;
    memcpy(&v0, body, 8);
    TEST_ASSERT_EQUAL_INT64(0, v0);

    /* Nothing pending now -- a second drain must report false, not stale data. */
    uint32_t n2, blen2; uint64_t t2; uint32_t s2;
    TEST_ASSERT_FALSE(capstream_drain(&cs, 1, body, sizeof(body), &n2, &t2, &s2, &blen2));
}

static void test_capstream_push_sequence_increments_per_chunk(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 10, 1, true);
    uint8_t body[PUSH_CHUNK_SAMPLES * 8];
    uint64_t first_tick; uint32_t seq;

    for (int32_t v = 0; v < (int32_t)PUSH_CHUNK_SAMPLES; v++) capstream_append(&cs, &v);
    drain_one(&cs, 1, body, sizeof(body), &first_tick, &seq);
    TEST_ASSERT_EQUAL_UINT32(0, seq);

    for (int32_t v = 0; v < (int32_t)PUSH_CHUNK_SAMPLES; v++) capstream_append(&cs, &v);
    drain_one(&cs, 1, body, sizeof(body), &first_tick, &seq);
    TEST_ASSERT_EQUAL_UINT32(1, seq);
    /* second chunk's first sample is logical index PUSH_CHUNK_SAMPLES, at period 10 */
    TEST_ASSERT_EQUAL_UINT64((uint64_t)PUSH_CHUNK_SAMPLES * 10, first_tick);
}

static void test_capstream_overflow_when_host_falls_behind(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 1, 1, true);
    /* Fill the first half, without draining it -- the host "isn't reading". */
    for (int32_t v = 0; v < (int32_t)PUSH_CHUNK_SAMPLES; v++) TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    TEST_ASSERT_TRUE(cs.pending[0]);
    TEST_ASSERT_FALSE(cs.overflowed);

    /* Filling the second half completely, with the first still undrained,
     * is the real overflow condition in the push model. */
    int32_t v;
    bool saw_overflow = false;
    for (uint32_t i = 0; i < PUSH_CHUNK_SAMPLES; i++) {
        v = (int32_t)i;
        if (!capstream_append(&cs, &v)) { saw_overflow = true; break; }
    }
    TEST_ASSERT_TRUE(saw_overflow);
    TEST_ASSERT_TRUE(cs.overflowed);
    capture_stream_t snapshot = cs;
    for (uint32_t i = 0; i < PUSH_CHUNK_SAMPLES * 3; i++) {
        TEST_ASSERT_FALSE(capstream_append(&cs, &v));
    }
    TEST_ASSERT_EQUAL_MEMORY(&snapshot, &cs, sizeof(cs));
}

static void test_capstream_flush_pushes_partial_final_chunk(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 10, 1, true);
    for (int32_t v = 0; v < 5; v++) capstream_append(&cs, &v); /* well under PUSH_CHUNK_SAMPLES */
    TEST_ASSERT_FALSE(cs.pending[0]);

    capstream_flush(&cs);
    TEST_ASSERT_TRUE(cs.pending[0]);

    uint8_t body[PUSH_CHUNK_SAMPLES * 8];
    uint64_t first_tick; uint32_t seq;
    uint32_t n = drain_one(&cs, 1, body, sizeof(body), &first_tick, &seq);
    TEST_ASSERT_EQUAL_UINT32(5, n);

    /* Idempotent: flushing again with nothing new appended must not
     * re-mark an already-drained, now-empty half as pending. */
    capstream_flush(&cs);
    TEST_ASSERT_FALSE(cs.pending[0]);
}

static void test_capstream_flush_drains_oldest_half_first(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 10, 1, true);
    uint8_t body[PUSH_CHUNK_SAMPLES * 8];
    uint64_t tick; uint32_t seq;
    int32_t v = 0;
    for (uint32_t i = 0; i < PUSH_CHUNK_SAMPLES; i++)
        TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    TEST_ASSERT_EQUAL_UINT32(PUSH_CHUNK_SAMPLES,
        drain_one(&cs, 1, body, sizeof(body), &tick, &seq));
    for (uint32_t i = 0; i < PUSH_CHUNK_SAMPLES; i++)
        TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    capstream_flush(&cs); /* older full half 1, newer short half 0 */
    TEST_ASSERT_TRUE(cs.pending[0] && cs.pending[1]);
    TEST_ASSERT_EQUAL_UINT32(PUSH_CHUNK_SAMPLES,
        drain_one(&cs, 1, body, sizeof(body), &tick, &seq));
    TEST_ASSERT_EQUAL_UINT64(PUSH_CHUNK_SAMPLES * 10, tick);
    TEST_ASSERT_EQUAL_UINT32(1, seq);
    TEST_ASSERT_EQUAL_UINT32(1, drain_one(&cs, 1, body, sizeof(body), &tick, &seq));
    TEST_ASSERT_EQUAL_UINT64(PUSH_CHUNK_SAMPLES * 20, tick);
    TEST_ASSERT_EQUAL_UINT32(2, seq);
}

static void test_capstream_multichannel_interleaving(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 1, 2, false);
    int32_t frame0[2] = {1, 100};
    int32_t frame1[2] = {2, 200};
    capstream_append(&cs, frame0);
    capstream_append(&cs, frame1);

    TEST_ASSERT_EQUAL_INT32(1, cs.staging[0][0]);
    TEST_ASSERT_EQUAL_INT32(100, cs.staging[0][1]);
    TEST_ASSERT_EQUAL_INT32(2, cs.staging[0][2]);
    TEST_ASSERT_EQUAL_INT32(200, cs.staging[0][3]);
}

static void test_capstream_hash_matches_reference_over_multiple_chunks(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 1, 1, true);
    uint8_t body[PUSH_CHUNK_SAMPLES * 8];
    uint64_t first_tick; uint32_t seq;

    /* 1.5 chunks' worth, so the drain path and the final flush both run,
     * exactly like a real run's tail. */
    uint32_t total = PUSH_CHUNK_SAMPLES + PUSH_CHUNK_SAMPLES / 2;
    sha256_ctx ref;
    sha256_init(&ref);
    for (uint32_t i = 0; i < total; i++) {
        int32_t v = (int32_t)i * 3 - 7;
        capstream_append(&cs, &v);
        uint64_t u = (uint64_t)(int64_t)v;
        uint8_t le[8];
        for (int b = 0; b < 8; b++) le[b] = (uint8_t)(u >> (8 * b));
        sha256_update(&ref, le, 8);
        if (cs.pending[0] || cs.pending[1]) drain_one(&cs, 1, body, sizeof(body), &first_tick, &seq);
    }
    capstream_flush(&cs);
    if (cs.pending[0] || cs.pending[1]) drain_one(&cs, 1, body, sizeof(body), &first_tick, &seq);

    uint8_t expected[32], actual[32];
    sha256_final(&ref, expected);
    capstream_final_hash(&cs, actual);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected, actual, 32);

    /* capstream_final_hash must not disturb the live running context --
     * calling it again (as a repeated GET_RUN would) gives the same digest. */
    uint8_t actual2[32];
    capstream_final_hash(&cs, actual2);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(actual, actual2, 32);
}

static void test_capstream_encode_chunk_delta_rle(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 1, 1, true);
    /* 0, 1, 2, 3 (constant delta +1), then 3, 3 (delta 0), padded to a
     * full chunk with a final run so the chunk actually flushes. */
    int32_t data[6] = {0, 1, 2, 3, 3, 3};
    for (int i = 0; i < 6; i++) capstream_append(&cs, &data[i]);
    capstream_flush(&cs);

    uint8_t out[64];
    uint64_t first_tick; uint32_t seq;
    uint32_t n = drain_one(&cs, 2 /* delta_rle_i32 */, out, sizeof(out), &first_tick, &seq);
    TEST_ASSERT_EQUAL_UINT32(6u, n);

    int64_t first;
    memcpy(&first, out, 8);
    TEST_ASSERT_EQUAL_INT64(0, first);

    uint32_t repeat; int32_t delta;
    memcpy(&repeat, out + 8, 4); memcpy(&delta, out + 12, 4);
    TEST_ASSERT_EQUAL_UINT32(3u, repeat);
    TEST_ASSERT_EQUAL_INT32(1, delta);

    memcpy(&repeat, out + 16, 4); memcpy(&delta, out + 20, 4);
    TEST_ASSERT_EQUAL_UINT32(2u, repeat);
    TEST_ASSERT_EQUAL_INT32(0, delta);
}

static void test_capstream_instance_bounds(void) {
    TEST_ASSERT_NOT_NULL(capstream_instance(0));
    TEST_ASSERT_NULL(capstream_instance(NUM_CAPTURE_STREAMS));
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_cmdbuf_put_and_complete);
    RUN_TEST(test_cmdbuf_rejects_out_of_declared_range);
    RUN_TEST(test_cmdbuf_rejects_value_out_of_i32_range);
    RUN_TEST(test_cmdbuf_rewrite_same_value_is_ok_conflict_is_rejected);
    RUN_TEST(test_cmdbuf_zero_declared_count_never_complete);
    RUN_TEST(test_cmdbuf_hash_canonical_matches_direct_sha256);
    RUN_TEST(test_capstream_append_counts_and_chunks_at_push_size);
    RUN_TEST(test_capstream_drain_reports_chunk_and_frees_half);
    RUN_TEST(test_capstream_push_sequence_increments_per_chunk);
    RUN_TEST(test_capstream_overflow_when_host_falls_behind);
    RUN_TEST(test_capstream_flush_pushes_partial_final_chunk);
    RUN_TEST(test_capstream_flush_drains_oldest_half_first);
    RUN_TEST(test_capstream_multichannel_interleaving);
    RUN_TEST(test_capstream_hash_matches_reference_over_multiple_chunks);
    RUN_TEST(test_capstream_encode_chunk_delta_rle);
    RUN_TEST(test_capstream_instance_bounds);
    return UNITY_END();
}
