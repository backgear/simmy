#pragma once
#include <stdint.h>

/* Compile-time capacities and pin map for the Blue Pill (STM32F103C8T6)
 * build of the axis characterization device. These are *this firmware's*
 * declared limits, not a universal default -- HELLO reports them and SEAL
 * enforces them. Tune to taste; everything downstream reads these symbols
 * rather than hardcoding numbers again.
 *
 * RAM budget (20KB total on C8): the command buffer is the one big
 * static allocation left; capture no longer buffers a whole run at all
 * (see run_buffer.h) -- each capture stream costs a fixed, tiny double
 * buffer instead, independent of run length, leaving headroom for the
 * USB CDC stack, ArduinoJson's working set (bounded by MAX_PAYLOAD
 * below) and the call stack.
 */

/* ---- Wire framing ----
 * The protocol's v1 *bootstrap* maximum is 4096 bytes, but it is only a
 * ceiling: "HELLO may negotiate a different maximum, never exceeding
 * either peer's supported value." With 20KB of total SRAM to share
 * across the reassembly buffer, the command/capture storage below, and
 * the USB/HAL/C-runtime's own static footprint, this firmware declares
 * (and enforces) a smaller max_payload in its own HELLO response rather
 * than reserve 4KB it mostly wouldn't use. */
#define AXIS_MAX_PAYLOAD        2048u
#define AXIS_HEADER_LEN         20u

/* ---- Command upload ---- */
#define MAX_COMMAND_SAMPLES     600u    /* ~2.4KB as int32_t + ~75B completeness bitmap */

/* ---- Capture ---- */
#define NUM_ENCODERS            2u      /* TIM2/TIM4 hardware encoder mode -- TIM3 is the PWM output below */
#define NUM_ANALOG_CHANNELS     2u      /* ADC1_IN4 (PA4), ADC1_IN5 (PA5) */
#define NUM_CAPTURE_STREAMS     (NUM_ENCODERS + 1u) /* one stream per encoder + one multi-channel analog stream */
/* Samples per pushed DATA chunk, per stream -- NOT a capture-length cap.
 * Capture is unsolicited-push streamed (see run_buffer.h); each stream's
 * only on-device storage is 2 of these chunks (ping-pong), so run length
 * is bounded by host/USB drain rate and run duration, not SRAM. */
#define PUSH_CHUNK_SAMPLES      32u

#define MAX_SEGMENTS            8u

/* ---- Tick base ----
 * TIM1 runs a 1 MHz tick and fires its update ISR every TICKS_PER_SERVICE
 * ticks. The original 5 us ISR starved USB while ARMED on the actual F103.
 * Use a conservative service rate; HELLO reports its timing quantization.
 * Physical acquisition aperture/jitter still require independent validation. */
#define TICK_HZ                 1000000ull
#define TICKS_PER_SERVICE       25u      /* 25 us -> 40 kHz service ISR */
#define ANALOG_MIN_PERIOD_TICKS 250u

/* ---- Pin map ----
 * TIM2/TIM4 CH1/CH2 pairs are reserved entirely for hardware quadrature
 * decode; TIM3 CH1/CH2 is reserved entirely for the RC-filtered PWM
 * analog output below (same TIM3/PA6/PA7 choice as the sibling
 * ~/analog-servo-f103 project's "poor-man's DAC" for this same part --
 * see PIN_PWM_POS/PIN_PWM_NEG). Step/dir/enable and the two ADC inputs
 * live on the remaining free pins. TIM1 is used only as the
 * service-tick timer (its own channel pins are left free). */
#define PIN_STEP                PA8   /* or quadrature A, mode-dependent */
#define PIN_DIR                 PA9   /* or quadrature B, mode-dependent */
#define PIN_ENABLE              PA10  /* drive enable, polarity from manifest */
#define PIN_ESTOP                PB0  /* active-low to GND, pulled up; optional */

#define PIN_ENC1_A               PB6   /* encoder 0, TIM4_CH1 */
#define PIN_ENC1_B               PB7   /* encoder 0, TIM4_CH2 */
#define PIN_ENC2_A               PA0   /* encoder 1, TIM2_CH1 */
#define PIN_ENC2_B               PA1   /* encoder 1, TIM2_CH2 */

#define PIN_ANALOG0              PA4   /* ADC1_IN4 */
#define PIN_ANALOG1              PA5   /* ADC1_IN5 */

/* RC-filtered PWM "poor-man's DAC": this part has no on-chip DAC at all
 * (see axis_hw.cpp's OUTPUT_ANALOG_PWM_RC handling). Bipolar effort is
 * encoded as two unipolar channels -- positive and negative each sweep
 * [0, TIM3_PWM_ARR] from their own external RC low-pass filter, and a
 * downstream differential-input stage takes their difference. Each
 * channel needs its own single-pole RC filter (e.g. ~1k/100nF for a
 * corner well below the carrier) between the pin and that stage. */
#define PIN_PWM_POS              PA6   /* TIM3_CH1 */
#define PIN_PWM_NEG              PA7   /* TIM3_CH2 */
#define PWM_TIMER_CLK_HZ         72000000ull  /* TIM3CLK: APB1 timer clock on this board's 72MHz SystemClock_Config */
#define TIM3_PWM_ARR             2249u        /* PSC=0 -> carrier = PWM_TIMER_CLK_HZ/(ARR+1) = exactly 32kHz */

/* Stream ids echoed in GET_RUN / carried in DATA frames. */
#define STREAM_ENCODER0          0u
#define STREAM_ENCODER1          1u
#define STREAM_ANALOG            NUM_ENCODERS
