#pragma once
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

void protocol_init(void);

/* Feeds raw bytes read off the USB CDC link into the frame reassembler.
 * Call from the main loop with whatever Serial.available() yields. */
void protocol_feed(const uint8_t *data, size_t len);

/* Services time-based transitions that don't need a host request: arming
 * timeout, and noticing axis_hw signaling RUNNING->COMPLETE/FAULT. Call
 * every main-loop iteration regardless of whether bytes arrived. */
void protocol_poll(void);

#ifdef __cplusplus
}
#endif
