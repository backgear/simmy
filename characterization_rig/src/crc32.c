#include "crc32.h"

static uint32_t table[256];
static int table_ready = 0;

static void build_table(void) {
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) {
            c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
        }
        table[i] = c;
    }
    table_ready = 1;
}

/* Matches zlib's crc32(): invert at entry/exit of every call, which makes
 * repeated calls compose exactly like one call over the concatenation --
 * same property Python's zlib.crc32(data, value) relies on. */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len) {
    if (!table_ready) build_table();
    uint32_t c = crc ^ 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        c = table[(c ^ data[i]) & 0xFFu] ^ (c >> 8);
    }
    return c ^ 0xFFFFFFFFu;
}
