#include <Arduino.h>
#include "protocol.h"

void setup() {
    /* PA15 is JTDI at reset. Free it without disabling PA13/PA14 SWD,
     * so ST-Link programming/debugging stays available. */
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    /* Optional PB12--PA12 jumper: pull D+ low without using a debug pin.
     * Open drain never drives D+ high. Without the jumper this pulse has
     * no effect on USB; a physical power cycle can still be necessary. */
    digitalWrite(PB12, LOW);
    pinMode(PB12, OUTPUT_OPEN_DRAIN);
    delay(100);
    pinMode(PB12, INPUT);
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
