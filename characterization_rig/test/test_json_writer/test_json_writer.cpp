#include <unity.h>
#include "json_writer.h"
#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* jw_finish() returns a length, not a NUL-terminated C string -- every
 * real caller in protocol.cpp sends exactly that many bytes rather than
 * strlen()'ing the buffer. NUL-terminate here (buffers below all have
 * headroom) so TEST_ASSERT_EQUAL_STRING can be used for readability. */
static size_t finish_and_terminate(json_writer_t *w) {
    size_t len = jw_finish(w);
    if (len > 0) w->buf[len] = '\0';
    return len;
}

static void test_flat_object(void) {
    char buf[128];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_str(&w, "op", "HELLO");
    jw_kv_i64(&w, "seq", 7);
    jw_kv_bool(&w, "ok", true);
    jw_kv_null(&w, "fault");
    jw_obj_end(&w);
    size_t len = finish_and_terminate(&w);

    TEST_ASSERT_TRUE(len > 0);
    TEST_ASSERT_EQUAL_STRING("{\"op\":\"HELLO\",\"seq\":7,\"ok\":true,\"fault\":null}", buf);
}

static void test_nested_object_and_array(void) {
    char buf[256];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_key(&w, "encoders");
    jw_arr_begin(&w);
    jw_i64(&w, 1);
    jw_i64(&w, 2);
    jw_i64(&w, 3);
    jw_arr_end(&w);
    jw_key(&w, "analog");
    jw_obj_begin(&w);
    jw_kv_u64(&w, "bits", 12);
    jw_obj_end(&w);
    jw_obj_end(&w);
    finish_and_terminate(&w);

    TEST_ASSERT_EQUAL_STRING("{\"encoders\":[1,2,3],\"analog\":{\"bits\":12}}", buf);
}

static void test_empty_array_and_object(void) {
    char buf[64];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_key(&w, "a");
    jw_arr_begin(&w);
    jw_arr_end(&w);
    jw_key(&w, "b");
    jw_obj_begin(&w);
    jw_obj_end(&w);
    jw_obj_end(&w);
    finish_and_terminate(&w);

    TEST_ASSERT_EQUAL_STRING("{\"a\":[],\"b\":{}}", buf);
}

static void test_string_escaping(void) {
    char buf[64];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_str(&w, "msg", "line\n\"quoted\"\\backslash");
    jw_obj_end(&w);
    finish_and_terminate(&w);

    TEST_ASSERT_EQUAL_STRING("{\"msg\":\"line\\n\\\"quoted\\\"\\\\backslash\"}", buf);
}

static void test_negative_and_large_integers(void) {
    char buf[128];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_i64(&w, "neg", -12345);
    jw_kv_i64(&w, "min64", INT64_MIN);
    jw_kv_u64(&w, "max64", UINT64_MAX);
    jw_obj_end(&w);
    finish_and_terminate(&w);

    TEST_ASSERT_EQUAL_STRING(
        "{\"neg\":-12345,\"min64\":-9223372036854775808,\"max64\":18446744073709551615}", buf);
}

static void test_overflow_is_reported_not_corrupted(void) {
    char buf[8]; /* far too small for even one key */
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_str(&w, "op", "HELLO_WITH_A_LONG_VALUE_THAT_OVERFLOWS");
    jw_obj_end(&w);
    size_t len = jw_finish(&w);

    TEST_ASSERT_EQUAL_UINT32(0u, (uint32_t)len);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_flat_object);
    RUN_TEST(test_nested_object_and_array);
    RUN_TEST(test_empty_array_and_object);
    RUN_TEST(test_string_escaping);
    RUN_TEST(test_negative_and_large_integers);
    RUN_TEST(test_overflow_is_reported_not_corrupted);
    return UNITY_END();
}
