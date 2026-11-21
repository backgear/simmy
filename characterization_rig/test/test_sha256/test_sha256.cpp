#include <unity.h>
#include "sha256.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void digest_hex(const char *msg, char out_hex[65]) {
    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, (const uint8_t *)msg, strlen(msg));
    uint8_t digest[32];
    sha256_final(&ctx, digest);
    sha256_hex(digest, out_hex);
}

static void test_empty_string(void) {
    char hex[65];
    digest_hex("", hex);
    TEST_ASSERT_EQUAL_STRING(
        "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855", hex);
}

static void test_known_vectors(void) {
    char hex[65];

    digest_hex("abc", hex);
    TEST_ASSERT_EQUAL_STRING(
        "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad", hex);
}

static void test_incremental_matches_one_shot(void) {
    const char *msg = "The quick brown fox jumps over the lazy dog";

    sha256_ctx one_shot_ctx;
    sha256_init(&one_shot_ctx);
    sha256_update(&one_shot_ctx, (const uint8_t *)msg, strlen(msg));
    uint8_t one_shot[32];
    sha256_final(&one_shot_ctx, one_shot);

    sha256_ctx inc_ctx;
    sha256_init(&inc_ctx);
    sha256_update(&inc_ctx, (const uint8_t *)msg, 9);
    sha256_update(&inc_ctx, (const uint8_t *)msg + 9, strlen(msg) - 9);
    uint8_t incremental[32];
    sha256_final(&inc_ctx, incremental);

    TEST_ASSERT_EQUAL_UINT8_ARRAY(one_shot, incremental, 32);
}

static void test_long_message_spans_multiple_blocks(void) {
    /* 1000 'a' bytes -- exercises the internal 64-byte block buffering
     * beyond a single sha256_update call. */
    char msg[1000];
    memset(msg, 'a', sizeof(msg));

    sha256_ctx ctx;
    sha256_init(&ctx);
    sha256_update(&ctx, (const uint8_t *)msg, sizeof(msg));
    uint8_t digest[32];
    sha256_final(&ctx, digest);
    char hex[65];
    sha256_hex(digest, hex);

    /* sha256sum of 1000 'a' bytes */
    TEST_ASSERT_EQUAL_STRING(
        "41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3", hex);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_empty_string);
    RUN_TEST(test_known_vectors);
    RUN_TEST(test_incremental_matches_one_shot);
    RUN_TEST(test_long_message_spans_multiple_blocks);
    return UNITY_END();
}
