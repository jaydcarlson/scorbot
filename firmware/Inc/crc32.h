#pragma once
#include <stdint.h>
#include <stddef.h>

/*
 * CRC32 (IEEE 802.3) — zlib compatible, stateful API
 *
 *  - Polynomial (reflected): 0xEDB88320
 *  - Initial value: 0xFFFFFFFF
 *  - Final XOR:    0xFFFFFFFF
 *  - Byte/bit order: identical to zlib.crc32()
 *
 * API:
 *   typedef struct { uint32_t crc; } crc32_ctx_t;
 *   void     crc32_init  (crc32_ctx_t *ctx);
 *   void     crc32_update(crc32_ctx_t *ctx, const void *data, size_t len);
 *   void     crc32_update_byte(crc32_ctx_t *ctx, uint8_t b); // optional helper
 *   uint32_t crc32_get   (const crc32_ctx_t *ctx);           // final value (zlib-style)
 *
 * Convenience one-shot:
 *   uint32_t crc32_calc(const void *data, size_t len);
 */

typedef struct {
    uint32_t crc;  // running (internal) CRC state
} crc32_ctx_t;


void crc32_init(crc32_ctx_t *ctx);
void crc32_update(crc32_ctx_t *ctx, const void *data, size_t len);

void crc32_update_byte(crc32_ctx_t *ctx, uint8_t b);
uint32_t crc32_get(const crc32_ctx_t *ctx);

uint32_t crc32_calc(const void *data, size_t len);

/* Optional: endian swap helper for RoCEv2 wire order */
uint32_t crc32_swap_bytes(uint32_t v);