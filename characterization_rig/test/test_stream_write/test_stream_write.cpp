#include <unity.h>
#include <string.h>
#include "stream_write.h"

void setUp(void) {}
void tearDown(void) {}

static void test_short_and_zero_writes_preserve_two_frames(void) {
    const uint8_t first[] = {1, 2, 3, 4, 5};
    const uint8_t second[] = {6, 7, 8};
    const size_t portions[] = {2, 0, 0, 1, 2, 0, 1, 2};
    uint8_t received[8] = {};
    size_t call = 0, len = 0, waits = 0;
    auto write = [&](const uint8_t *p, size_t n) -> size_t {
        TEST_ASSERT_TRUE(call < sizeof(portions) / sizeof(portions[0]));
        size_t portion = portions[call++];
        TEST_ASSERT_TRUE(portion <= n);
        memcpy(received + len, p, portion);
        len += portion;
        return portion;
    };
    auto wait = [&]() { waits++; };
    stream_write_all(first, sizeof(first), write, wait);
    stream_write_all(second, sizeof(second), write, wait);
    const uint8_t expected[] = {1, 2, 3, 4, 5, 6, 7, 8};
    TEST_ASSERT_EQUAL_UINT32(8, len);
    TEST_ASSERT_EQUAL_UINT32(3, waits);
    TEST_ASSERT_EQUAL_MEMORY(expected, received, sizeof(expected));
}

static void test_empty_write_does_not_touch_transport(void) {
    auto write = [](const uint8_t *, size_t) -> size_t {
        TEST_FAIL_MESSAGE("unexpected write");
        return 0;
    };
    auto wait = []() { TEST_FAIL_MESSAGE("unexpected wait"); };
    stream_write_all(nullptr, 0, write, wait);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_short_and_zero_writes_preserve_two_frames);
    RUN_TEST(test_empty_write_does_not_touch_transport);
    return UNITY_END();
}
