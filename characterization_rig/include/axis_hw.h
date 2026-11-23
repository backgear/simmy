#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "axis_config.h"
#include "axis_pid.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    OUTPUT_STEP_DIR = 0,
    OUTPUT_QUADRATURE_COMMAND = 1,
    /* RC-filtered PWM "poor-man's DAC" (this part has no on-chip DAC):
     * TIM3 CH1/CH2 on PIN_PWM_POS/PIN_PWM_NEG, bipolar effort split into
     * two unipolar channels. Each command sample is the *held* signed
     * code for that interval (zero-order hold at the interval's start),
     * not a cumulative digital count -- see axis_hw.cpp. */
    OUTPUT_ANALOG_PWM_RC = 2,
    OUTPUT_ANALOG_POSITION_SERVO = 3, /* references in encoder counts; live 1kHz Control.c */
} output_mode_t;
static inline bool axis_output_is_pwm(output_mode_t m) {
    return m == OUTPUT_ANALOG_PWM_RC || m == OUTPUT_ANALOG_POSITION_SERVO;
}

typedef enum {
    CONN_SUPERVISED = 0,
    CONN_AUTONOMOUS = 1,
} connection_policy_t;

typedef enum {
    SAFE_DISABLE_DRIVE = 0,  /* deassert PIN_ENABLE, leave step/dir idle */
    SAFE_ZERO_OUTPUT = 1,    /* hold step/dir at configured idle level, drive stays enabled */
    SAFE_STOP_PULSES = 2,    /* stop generating edges, enable pin untouched */
} safe_action_t;

/* Validated, device-supported subset of a BEGIN manifest. protocol.cpp
 * fills this in from JSON and rejects anything this firmware cannot
 * honor (UNSUPPORTED/RANGE) rather than silently ignoring it. */
typedef struct {
    output_mode_t output_mode;
    uint32_t command_sample_count;
    uint64_t command_period_ticks;
    uint64_t capture_pre_roll_ticks;
    uint64_t capture_post_roll_ticks;

    bool output_idle_level_high;
    bool enable_active_high;
    bool output_enable_used;
    uint32_t step_pulse_width_ticks;
    uint32_t dir_setup_ticks;
    uint32_t dir_hold_ticks;
    bool quadrature_invert;
    int32_t analog_output_full_scale;   /* OUTPUT_ANALOG_PWM_RC only: command code magnitude mapping to full PWM swing */

    bool encoder_enabled[NUM_ENCODERS];
    uint8_t encoder_decode_mult[NUM_ENCODERS];  /* 1, 2, or 4 */
    bool encoder_invert[NUM_ENCODERS];
    bool travel_limit_enabled[NUM_ENCODERS];
    int32_t travel_limit_counts[NUM_ENCODERS];  /* +/- from latched origin */
    uint64_t encoder_sample_period_ticks;

    bool analog_enabled;             /* both analog channels captured together as one stream */
    uint64_t analog_sample_period_ticks;

    uint64_t watchdog_timeout_ticks; /* supervised only */
    uint64_t arming_timeout_ticks;
    connection_policy_t connection_policy;
    safe_action_t safe_action;
    axis_pid_config_t servo;
} run_config_t;
static inline bool axis_stream_enabled(const run_config_t *c, uint32_t s) {
    return s < NUM_ENCODERS ? c->encoder_enabled[s] :
        (c->analog_enabled || c->output_mode == OUTPUT_ANALOG_POSITION_SERVO);
}

typedef enum {
    FAULT_NONE = 0,
    FAULT_HOST_ABORT,
    FAULT_WATCHDOG,
    FAULT_INTERLOCK,
    FAULT_OVERFLOW,
    FAULT_CAPTURE_TIMING,
    FAULT_ESTOP,
} fault_reason_t;

/* Hardware setup: timers, GPIO, ADC. Call once from setup(). */
void axis_hw_init(void);

/* Monotonic device tick counter (TICK_HZ), safe to call from either
 * context; internally guards the 64-bit read against the ISR. */
uint64_t axis_hw_now_ticks(void);

bool axis_hw_estop_asserted(void);

/* Drives all outputs to their powered-up idle/safe condition. Called at
 * boot and after CLEAR. */
void axis_hw_set_idle_outputs(void);

/* Validates `cfg` against real hardware limits (pin/timer capacity,
 * decode multiplier support, etc.) independent of manifest-shape
 * validation already done in protocol.cpp. Returns an error code string
 * (static, no ownership) or NULL if acceptable. */
const char *axis_hw_validate_config(const run_config_t *cfg);

/* Applies `cfg`: configures encoder timers/decode, analog channel
 * selection, output pin roles -- but does not arm or move anything. */
void axis_hw_configure(const run_config_t *cfg);

/* ARM: latches encoder origins (software baseline for travel limits),
 * leaves outputs at idle. */
void axis_hw_latch_origin(void);

/* START: schedules command/capture execution to begin at the given
 * absolute device ticks (already chosen by protocol.cpp to include
 * pre-roll and a safety margin past "now"). */
void axis_hw_schedule_start(uint64_t command_start_tick, uint64_t capture_start_tick);

/* Immediately stops generation/capture and applies cfg->safe_action. */
void axis_hw_abort(fault_reason_t reason);

/* Release terminal/armed hardware state with outputs disabled. */
void axis_hw_clear_run(void);

/* Renews the watchdog lease (KEEPALIVE). */
void axis_hw_feed_watchdog(void);

/* ---- status snapshot, read by STATUS/GET_RUN handlers ---- */
typedef struct {
    bool running;
    bool armed;
    bool complete;
    bool fault;
    fault_reason_t fault_reason;
    uint64_t fault_tick;
    uint32_t executed_command_index;
    uint32_t captured_count[NUM_CAPTURE_STREAMS];
    bool capture_complete;
    bool output_enabled;
    uint64_t command_start_tick;
    uint64_t capture_start_tick;
    uint64_t run_end_tick;
} axis_status_t;

void axis_hw_get_status(axis_status_t *out);

/* Direct access to this run's capture buffers, valid after BEGIN. */
struct capture_stream_s; /* fwd decl, real type in run_buffer.h */
void *axis_hw_capture_stream(uint32_t stream_id); /* returns capture_stream_t* */

#ifdef __cplusplus
}
#endif
