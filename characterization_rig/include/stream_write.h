#pragma once
#include <stddef.h>
#include <stdint.h>

/* Keep one frame's byte order across short/zero transport writes. The caller
 * must not interleave another frame. wait() must leave realtime interrupts
 * enabled so local fault handling does not depend on transport progress.
 * A permanently disconnected peer can stall this main-thread operation;
 * local watchdog/overflow shutdown must remain independent of it.
 */
template <typename Write, typename Wait>
void stream_write_all(const uint8_t *data, size_t len, Write write, Wait wait) {
    while (len) {
        size_t n = write(data, len);
        if (n) {
            data += n;
            len -= n;
        } else {
            wait();
        }
    }
}
