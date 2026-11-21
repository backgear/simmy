#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* CRC-32/ISO-HDLC, matching Python's zlib.crc32(). seed with 0, feed bytes
 * incrementally (header then payload), use the returned value directly --
 * it already includes the final XOR. */
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
