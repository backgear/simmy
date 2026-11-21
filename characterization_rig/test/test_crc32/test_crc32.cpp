#include <unity.h>
#include "crc32.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

static void test_empty_is_zero(void) {
    TEST_ASSERT_EQUAL_UINT32(0u, crc32_update(0u, NULL, 0));
}

static void test_matches_known_zlib_vector(void) {
    /* zlib.crc32(b"123456789") == 0xCBF43926 */
    const uint8_t data[] = "123456789";
    uint32_t crc = crc32_update(0u, data, 9);
    TEST_ASSERT_EQUAL_UINT32(0xCBF43926u, crc);
}

static void test_matches_known_vector_the_quick_brown_fox(void) {
    /* zlib.crc32(b"The quick brown fox jumps over the lazy dog") == 0x414FA339 */
    const char *s = "The quick brown fox jumps over the lazy dog";
    uint32_t crc = crc32_update(0u, (const uint8_t *)s, strlen(s));
    TEST_ASSERT_EQUAL_UINT32(0x414FA339u, crc);
}

static void test_incremental_matches_one_shot(void) {
    const char *s = "The quick brown fox jumps over the lazy dog";
    size_t n = strlen(s);
    uint32_t one_shot = crc32_update(0u, (const uint8_t *)s, n);

    uint32_t incremental = 0u;
    incremental = crc32_update(incremental, (const uint8_t *)s, 10);
    incremental = crc32_update(incremental, (const uint8_t *)s + 10, n - 10);

    TEST_ASSERT_EQUAL_UINT32(one_shot, incremental);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_empty_is_zero);
    RUN_TEST(test_matches_known_zlib_vector);
    RUN_TEST(test_matches_known_vector_the_quick_brown_fox);
    RUN_TEST(test_incremental_matches_one_shot);
    return UNITY_END();
}
