#pragma once
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t state[8];
    uint64_t bitlen;
    uint8_t buf[64];
    size_t buflen;
} sha256_ctx;

void sha256_init(sha256_ctx *ctx);
void sha256_update(sha256_ctx *ctx, const uint8_t *data, size_t len);
/* Writes 32 raw digest bytes to out. */
void sha256_final(sha256_ctx *ctx, uint8_t out[32]);
/* Writes 64 lowercase hex chars + NUL to out (out must hold >=65 bytes). */
void sha256_hex(const uint8_t digest[32], char out[65]);

#ifdef __cplusplus
}
#endif
