#include <Arduino.h>
#include "protocol.h"

void setup() {
    Serial.begin(115200); /* baud is irrelevant over USB CDC, kept for clarity */
    protocol_init();
}

void loop() {
    uint8_t chunk[256];
    size_t n = 0;
    while (Serial.available() && n < sizeof(chunk)) {
        chunk[n++] = (uint8_t)Serial.read();
    }
    if (n) protocol_feed(chunk, n);
    protocol_poll();
}
