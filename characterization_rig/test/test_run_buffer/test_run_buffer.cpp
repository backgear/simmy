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

/* ------------------------- capture streams ------------------------- */

static void test_capstream_append_and_overflow(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 2, 100, 1);
    int32_t v = 11;
    TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    v = 22;
    TEST_ASSERT_TRUE(capstream_append(&cs, &v));
    TEST_ASSERT_EQUAL_UINT32(2u, cs.count);
    TEST_ASSERT_FALSE(cs.overflowed);

    v = 33;
    TEST_ASSERT_FALSE(capstream_append(&cs, &v)); /* capacity exceeded */
    TEST_ASSERT_TRUE(cs.overflowed);
    TEST_ASSERT_EQUAL_UINT32(2u, cs.count); /* unchanged */
}

static void test_capstream_reset_clamps_capacity_by_channels(void) {
    capture_stream_t cs;
    /* Requesting more than MAX_CAPTURE_SAMPLES/channels must clamp, not
     * overrun the fixed samples[] array (2 channels -> half the slots). */
    capstream_reset(&cs, MAX_CAPTURE_SAMPLES, 1, 2);
    TEST_ASSERT_EQUAL_UINT32(MAX_CAPTURE_SAMPLES / 2, cs.capacity);
}

static void test_capstream_multichannel_interleaving(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 3, 1, 2);
    int32_t frame0[2] = {1, 100};
    int32_t frame1[2] = {2, 200};
    TEST_ASSERT_TRUE(capstream_append(&cs, frame0));
    TEST_ASSERT_TRUE(capstream_append(&cs, frame1));

    TEST_ASSERT_EQUAL_INT32(1, cs.samples[0]);
    TEST_ASSERT_EQUAL_INT32(100, cs.samples[1]);
    TEST_ASSERT_EQUAL_INT32(2, cs.samples[2]);
    TEST_ASSERT_EQUAL_INT32(200, cs.samples[3]);
}

static void test_capstream_hash_widened_vs_raw(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 2, 1, 1);
    int32_t v = -5;
    capstream_append(&cs, &v);
    v = 7;
    capstream_append(&cs, &v);

    uint8_t digest_i32[32], digest_i64[32];
    capstream_hash(&cs, false, digest_i32);
    capstream_hash(&cs, true, digest_i64);

    /* Different element widths must produce different hashes over the
     * same logical values. */
    TEST_ASSERT_NOT_EQUAL(0, memcmp(digest_i32, digest_i64, 32));
}

static void test_capstream_encode_block_raw_i32(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 4, 1, 1);
    for (int32_t v = 10; v < 14; v++) capstream_append(&cs, &v);

    uint8_t out[64];
    uint32_t n = capstream_encode_block(&cs, 0, 2, 3 /* raw_i32 */, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(2u * 4u, n); /* 2 samples * 4 bytes */
    int32_t v0, v1;
    memcpy(&v0, out, 4);
    memcpy(&v1, out + 4, 4);
    TEST_ASSERT_EQUAL_INT32(10, v0);
    TEST_ASSERT_EQUAL_INT32(11, v1);

    /* second block */
    n = capstream_encode_block(&cs, 1, 2, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(8u, n);
    memcpy(&v0, out, 4);
    memcpy(&v1, out + 4, 4);
    TEST_ASSERT_EQUAL_INT32(12, v0);
    TEST_ASSERT_EQUAL_INT32(13, v1);
}

static void test_capstream_encode_block_past_end_returns_zero(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 4, 1, 1);
    for (int32_t v = 0; v < 2; v++) capstream_append(&cs, &v);

    uint8_t out[64];
    /* block_index=1, block_len=2 -> start=2, but count is only 2 */
    uint32_t n = capstream_encode_block(&cs, 1, 2, 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(0u, n);
}

static void test_capstream_encode_block_delta_rle(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 6, 1, 1);
    /* 0, 1, 2, 3 (constant delta +1), then 3, 3 (delta 0) */
    int32_t data[6] = {0, 1, 2, 3, 3, 3};
    for (int i = 0; i < 6; i++) capstream_append(&cs, &data[i]);

    uint8_t out[64];
    uint32_t n = capstream_encode_block(&cs, 0, 6, 2 /* delta_rle_i32 */, out, sizeof(out));
    TEST_ASSERT_TRUE(n > 0);

    /* First 8 bytes: initial value as i64 little-endian = 0 */
    int64_t first;
    memcpy(&first, out, 8);
    TEST_ASSERT_EQUAL_INT64(0, first);

    /* Next run: repeat=3 (deltas for samples 1,2,3 vs their predecessor
     * are all +1), delta=1 */
    uint32_t repeat;
    int32_t delta;
    memcpy(&repeat, out + 8, 4);
    memcpy(&delta, out + 12, 4);
    TEST_ASSERT_EQUAL_UINT32(3u, repeat);
    TEST_ASSERT_EQUAL_INT32(1, delta);

    /* Final run: samples 4,5 both delta 0 vs predecessor */
    memcpy(&repeat, out + 16, 4);
    memcpy(&delta, out + 20, 4);
    TEST_ASSERT_EQUAL_UINT32(2u, repeat);
    TEST_ASSERT_EQUAL_INT32(0, delta);

    TEST_ASSERT_EQUAL_UINT32(24u, n);
}

static void test_capstream_encode_block_delta_rle_rejects_multichannel(void) {
    capture_stream_t cs;
    capstream_reset(&cs, 4, 1, 2);
    int32_t frame[2] = {1, 2};
    capstream_append(&cs, frame);

    uint8_t out[64];
    uint32_t n = capstream_encode_block(&cs, 0, 1, 2, out, sizeof(out));
    TEST_ASSERT_EQUAL_UINT32(0u, n);
}

/* ------------------------- singletons ------------------------- */

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
    RUN_TEST(test_capstream_append_and_overflow);
    RUN_TEST(test_capstream_reset_clamps_capacity_by_channels);
    RUN_TEST(test_capstream_multichannel_interleaving);
    RUN_TEST(test_capstream_hash_widened_vs_raw);
    RUN_TEST(test_capstream_encode_block_raw_i32);
    RUN_TEST(test_capstream_encode_block_past_end_returns_zero);
    RUN_TEST(test_capstream_encode_block_delta_rle);
    RUN_TEST(test_capstream_encode_block_delta_rle_rejects_multichannel);
    RUN_TEST(test_capstream_instance_bounds);
    return UNITY_END();
}
