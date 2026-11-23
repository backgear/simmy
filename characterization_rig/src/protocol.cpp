#include <Arduino.h>
#include <ArduinoJson.h>
#include <string.h>
#include <stdlib.h>

#include "protocol.h"
#include "axis_config.h"
#include "axis_hw.h"
#include "source_id.h"
#include "run_buffer.h"
#include "crc32.h"
#include "sha256.h"
#include "json_writer.h"
#include "stream_write.h"

/* ===========================================================================
 * Framing + JSON dispatch for docs/axis_device_protocol.md.
 *
 * Simplifications this firmware makes, deliberately and documented (the
 * protocol explicitly allows a device to decline what it cannot honor):
 *
 *  - Manifest numbers must be JSON integers; a float is UNSUPPORTED. This
 *    keeps "canonical JSON" tractable on an MCU: instead of a generic
 *    RFC 8785 canonicalizer over arbitrary input, the retained manifest
 *    hash and the "manifest" echoed in GET_RUN are both this firmware's
 *    own normalized reconstruction of the accepted fields (fixed key
 *    order, see build_canonical_manifest). A host that wants to verify
 *    the hash should canonicalize that same accepted-field set, not its
 *    original request bytes.
 *  - One encoder decode multiplier (x4, hardware TI12 mode) and one
 *    output electrical idle/enable model. See axis_hw.cpp's file header
 *    for the complete list of what maps to real silicon here.
 *  - SEAL requires every command interval's edge rate to need at most
 *    one emitted edge per service tick (TICKS_PER_SERVICE), so the
 *    runtime step generator never needs a catch-up loop. HELLO reports
 *    the service tick rate so a host can size its sweeps accordingly.
 * ==========================================================================
 */

#define SCHEDULE_MARGIN_TICKS (200ull * TICKS_PER_SERVICE)   /* ~1ms headroom before START's chosen start tick */
#define RESPONSE_BUF_LEN       (AXIS_MAX_PAYLOAD + 256u)

typedef enum {
    ST_IDLE = 0, ST_UPLOADING, ST_READY, ST_ARMED, ST_RUNNING, ST_COMPLETE, ST_FAULT
} device_state_t;

static device_state_t g_state = ST_IDLE;
static uint32_t g_boot_id;
static char g_device_serial[25];

static char g_run_id[40];
static bool g_have_run = false;
static run_config_t g_rc;
static char g_drive_interpretation[16] = "position";

typedef struct { char name[20]; uint32_t start; uint32_t end; } segment_t;
static segment_t g_segments[MAX_SEGMENTS];
static uint32_t g_segment_count = 0;

static char g_manifest_hash_hex[65];
static char g_command_hash_hex[65];
static uint64_t g_arm_deadline = 0;
static bool g_flushed_this_run = false; /* final partial chunk pushed exactly once per run */

/* ------------------------------- framing -------------------------------- */

#define RXCAP (AXIS_HEADER_LEN + AXIS_MAX_PAYLOAD + 64u)
static uint8_t g_rx[RXCAP];
static size_t g_rx_len = 0;

static void handle_request_json(uint32_t seq, const char *json, size_t len);

static void send_frame(uint8_t kind, uint32_t seq, const uint8_t *payload, uint32_t payload_len) {
    uint8_t header[AXIS_HEADER_LEN];
    memcpy(header, "AXIS", 4);
    header[4] = 1;        /* version */
    header[5] = kind;
    header[6] = 0; header[7] = 0; /* flags */
    header[8]  = (uint8_t)(seq); header[9] = (uint8_t)(seq >> 8);
    header[10] = (uint8_t)(seq >> 16); header[11] = (uint8_t)(seq >> 24);
    header[12] = (uint8_t)(payload_len); header[13] = (uint8_t)(payload_len >> 8);
    header[14] = (uint8_t)(payload_len >> 16); header[15] = (uint8_t)(payload_len >> 24);

    uint32_t crc = crc32_update(0, header, 16);
    crc = crc32_update(crc, payload, payload_len);
    header[16] = (uint8_t)(crc); header[17] = (uint8_t)(crc >> 8);
    header[18] = (uint8_t)(crc >> 16); header[19] = (uint8_t)(crc >> 24);

    /* USB CDC may return short/zero after its transmit timeout. Never
     * start the next frame while this one's bytes are still outstanding.
     * Interrupts stay enabled: watchdog/overflow still stop outputs locally. */
    auto write = [](const uint8_t *p, size_t n) { return Serial.write(p, n); };
    auto wait = []() { delay(1); };
    stream_write_all(header, AXIS_HEADER_LEN, write, wait);
    stream_write_all(payload, payload_len, write, wait);
}

static void try_parse_frames(void) {
    for (;;) {
        if (g_rx_len < AXIS_HEADER_LEN) return;

        size_t magic_at = (size_t)-1;
        for (size_t i = 0; i + 4 <= g_rx_len; i++) {
            if (memcmp(g_rx + i, "AXIS", 4) == 0) { magic_at = i; break; }
        }
        if (magic_at == (size_t)-1) {
            /* Keep a tail in case "AXIS" straddles this call's boundary. */
            size_t keep = (g_rx_len > 3) ? 3 : g_rx_len;
            memmove(g_rx, g_rx + g_rx_len - keep, keep);
            g_rx_len = keep;
            return;
        }
        if (magic_at > 0) {
            memmove(g_rx, g_rx + magic_at, g_rx_len - magic_at);
            g_rx_len -= magic_at;
        }
        if (g_rx_len < AXIS_HEADER_LEN) return;

        uint8_t version = g_rx[4];
        uint8_t kind = g_rx[5];
        uint16_t flags = (uint16_t)(g_rx[6] | (g_rx[7] << 8));
        if (version != 1 || flags != 0 || kind < 1 || kind > 4) {
            memmove(g_rx, g_rx + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }
        uint32_t seq = (uint32_t)g_rx[8] | ((uint32_t)g_rx[9] << 8) |
                       ((uint32_t)g_rx[10] << 16) | ((uint32_t)g_rx[11] << 24);
        uint32_t payload_len = (uint32_t)g_rx[12] | ((uint32_t)g_rx[13] << 8) |
                                ((uint32_t)g_rx[14] << 16) | ((uint32_t)g_rx[15] << 24);
        if (payload_len > AXIS_MAX_PAYLOAD) {
            memmove(g_rx, g_rx + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }
        if (g_rx_len < AXIS_HEADER_LEN + payload_len) return; /* wait for more bytes */

        uint32_t stored_crc = (uint32_t)g_rx[16] | ((uint32_t)g_rx[17] << 8) |
                               ((uint32_t)g_rx[18] << 16) | ((uint32_t)g_rx[19] << 24);
        uint32_t crc = crc32_update(0, g_rx, 16);
        crc = crc32_update(crc, g_rx + AXIS_HEADER_LEN, payload_len);
        if (crc != stored_crc) {
            memmove(g_rx, g_rx + 1, g_rx_len - 1);
            g_rx_len -= 1;
            continue;
        }

        if (kind == 1 /* REQUEST */) {
            handle_request_json(seq, (const char *)(g_rx + AXIS_HEADER_LEN), payload_len);
        }

        size_t consumed = AXIS_HEADER_LEN + payload_len;
        memmove(g_rx, g_rx + consumed, g_rx_len - consumed);
        g_rx_len -= consumed;
    }
}

void protocol_feed(const uint8_t *data, size_t len) {
    while (len > 0) {
        size_t space = RXCAP - g_rx_len;
        if (space == 0) {
            /* Reassembly stalled on garbage; drop the oldest half to recover
             * rather than wedging forever on an unparseable backlog. */
            size_t drop = RXCAP / 2;
            memmove(g_rx, g_rx + drop, g_rx_len - drop);
            g_rx_len -= drop;
            space = RXCAP - g_rx_len;
        }
        size_t take = (len < space) ? len : space;
        memcpy(g_rx + g_rx_len, data, take);
        g_rx_len += take;
        data += take;
        len -= take;
        try_parse_frames();
    }
}

/* -------------------------------- helpers -------------------------------- */

static bool parse_i64_strict(const char *s, int64_t *out) {
    if (!s || !*s) return false;
    char *end = nullptr;
    errno = 0;
    long long v = strtoll(s, &end, 10);
    if (errno != 0 || end == s || *end != '\0') return false;
    *out = (int64_t)v;
    return true;
}

static void send_error(uint32_t seq, const char *op, const char *code, const char *detail) {
    char buf[320];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", false);
    jw_kv_str(&w, "op", op);
    jw_kv_str(&w, "error", code);
    if (detail) jw_kv_str(&w, "detail", detail);
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

static bool check_run_id(JsonVariantConst doc, uint32_t seq, const char *op) {
    const char *rid = doc["run_id"] | (const char *)nullptr;
    if (!rid) { send_error(seq, op, "RUN_ID", "missing run_id"); return false; }
    if (!g_have_run || strncmp(rid, g_run_id, sizeof(g_run_id)) != 0) {
        send_error(seq, op, "RUN_ID", "run_id does not match the active run");
        return false;
    }
    return true;
}

static void fault_from_hw(void) {
    axis_status_t st;
    axis_hw_get_status(&st);
    bool was_running = (g_state == ST_RUNNING);
    if (st.fault && g_state != ST_FAULT) g_state = ST_FAULT;
    else if (st.complete && g_state == ST_RUNNING) g_state = ST_COMPLETE;

    /* Push streaming leaves the final, possibly-partial chunk sitting in
     * the active half with nothing to ever fill it the rest of the way --
     * flush it once, right as the run reaches a terminal state. */
    if (was_running && (g_state == ST_FAULT || g_state == ST_COMPLETE) && !g_flushed_this_run) {
        for (uint32_t s = 0; s < NUM_CAPTURE_STREAMS; s++) capstream_flush(capstream_instance(s));
        g_flushed_this_run = true;
    }
}

/* --------------------------- canonical manifest --------------------------- */
static const char *output_mode_name(output_mode_t m) {
    switch (m) {
        case OUTPUT_STEP_DIR: return "step_dir";
        case OUTPUT_QUADRATURE_COMMAND: return "quadrature_command";
        case OUTPUT_ANALOG_PWM_RC: return "analog_effort";
        case OUTPUT_ANALOG_POSITION_SERVO: return "analog_position_servo";
    }
    return "step_dir";
}

/* This firmware's own fixed-key-order reconstruction of the accepted
 * manifest fields -- see the file header note on why this stands in for
 * a generic RFC 8785 canonicalizer. */
static void manifest_hash_sink(void *ctx, const char *bytes, size_t len) {
    sha256_update((sha256_ctx *)ctx, (const uint8_t *)bytes, len);
}

static size_t build_canonical_manifest(sha256_ctx *ctx) {
    json_writer_t w;
    jw_init_sink(&w, manifest_hash_sink, ctx);
    jw_obj_begin(&w);
    jw_kv_str(&w, "drive_interpretation", g_drive_interpretation);
    jw_kv_str(&w, "output_mode", output_mode_name(g_rc.output_mode));
    jw_kv_u64(&w, "command_sample_count", g_rc.command_sample_count);
    jw_kv_u64(&w, "command_period_ticks", g_rc.command_period_ticks);
    jw_kv_u64(&w, "capture_pre_roll_ticks", g_rc.capture_pre_roll_ticks);
    jw_kv_u64(&w, "capture_post_roll_ticks", g_rc.capture_post_roll_ticks);
    jw_kv_bool(&w, "output_idle_level_high", g_rc.output_idle_level_high);
    jw_kv_bool(&w, "enable_active_high", g_rc.enable_active_high);
    jw_kv_bool(&w, "output_enable_used", g_rc.output_enable_used);
    jw_kv_u64(&w, "step_pulse_width_ticks", g_rc.step_pulse_width_ticks);
    jw_kv_u64(&w, "dir_setup_ticks", g_rc.dir_setup_ticks);
    jw_kv_u64(&w, "dir_hold_ticks", g_rc.dir_hold_ticks);
    jw_kv_bool(&w, "quadrature_invert", g_rc.quadrature_invert);
    jw_kv_i64(&w, "analog_output_full_scale", g_rc.analog_output_full_scale);
    if (g_rc.output_mode == OUTPUT_ANALOG_POSITION_SERVO) {
        jw_kv_i64(&w, "servo_position_kp_q8", g_rc.servo.position_kp_q8);
        jw_kv_i64(&w, "servo_velocity_kp_q8", g_rc.servo.velocity_kp_q8);
        jw_kv_i64(&w, "servo_velocity_ki_q8", g_rc.servo.velocity_ki_q8);
        jw_kv_i64(&w, "servo_torque_limit_code", g_rc.servo.torque_limit_code);
        jw_kv_i64(&w, "servo_error_limit_counts", g_rc.servo.error_limit_counts);
        jw_kv_i64(&w, "servo_velocity_clamp", g_rc.servo.velocity_clamp);
    }

    jw_key(&w, "encoders");
    jw_arr_begin(&w);
    for (uint32_t i = 0; i < NUM_ENCODERS; i++) {
        jw_obj_begin(&w);
        jw_kv_bool(&w, "enabled", g_rc.encoder_enabled[i]);
        jw_kv_u64(&w, "decode_mult", g_rc.encoder_decode_mult[i]);
        jw_kv_bool(&w, "invert", g_rc.encoder_invert[i]);
        jw_kv_bool(&w, "travel_limit_enabled", g_rc.travel_limit_enabled[i]);
        jw_kv_i64(&w, "travel_limit_counts", g_rc.travel_limit_counts[i]);
        jw_obj_end(&w);
    }
    jw_arr_end(&w);
    jw_kv_u64(&w, "encoder_sample_period_ticks", g_rc.encoder_sample_period_ticks);

    jw_kv_bool(&w, "analog_enabled", g_rc.analog_enabled);
    jw_kv_u64(&w, "analog_sample_period_ticks", g_rc.analog_sample_period_ticks);

    jw_kv_u64(&w, "watchdog_timeout_ticks", g_rc.watchdog_timeout_ticks);
    jw_kv_u64(&w, "arming_timeout_ticks", g_rc.arming_timeout_ticks);
    jw_kv_str(&w, "connection_policy", g_rc.connection_policy == CONN_SUPERVISED ? "supervised" : "autonomous");
    jw_kv_str(&w, "safe_action", g_rc.safe_action == SAFE_DISABLE_DRIVE ? "disable_drive" :
                                  g_rc.safe_action == SAFE_ZERO_OUTPUT ? "zero_output" : "stop_pulses");

    jw_key(&w, "segments");
    jw_arr_begin(&w);
    for (uint32_t i = 0; i < g_segment_count; i++) {
        jw_obj_begin(&w);
        jw_kv_str(&w, "name", g_segments[i].name);
        jw_kv_u64(&w, "start", g_segments[i].start);
        jw_kv_u64(&w, "end", g_segments[i].end);
        jw_obj_end(&w);
    }
    jw_arr_end(&w);

    jw_obj_end(&w);
    return jw_finish(&w);
}

/* ---------------------------------- HELLO ---------------------------------- */

static void handle_HELLO(uint32_t seq) {
    char buf[RESPONSE_BUF_LEN];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "HELLO");
    jw_kv_u64(&w, "version", 1);
    jw_kv_str(&w, "device_serial", g_device_serial);
    jw_kv_str(&w, "firmware_version", "0.2.0");
    jw_kv_u64(&w, "boot_id", g_boot_id);
    jw_kv_u64(&w, "tick_hz", TICK_HZ);
    jw_kv_u64(&w, "tick_width", 64);
    jw_kv_u64(&w, "max_payload", AXIS_MAX_PAYLOAD);

    jw_key(&w, "output_modes");
    jw_arr_begin(&w);
    jw_str(&w, "step_dir"); jw_str(&w, "quadrature_command"); jw_str(&w, "analog_effort");
    jw_str(&w, "analog_position_servo");
    jw_arr_end(&w);

    jw_key(&w, "step_dir");
    jw_obj_begin(&w);
    jw_kv_u64(&w, "step_pulse_width_min_ticks", TICKS_PER_SERVICE);
    jw_kv_u64(&w, "dir_setup_min_ticks", 0);
    jw_kv_u64(&w, "dir_hold_min_ticks", 0);
    jw_obj_end(&w);

    jw_key(&w, "analog_effort");
    jw_obj_begin(&w);
    jw_kv_str(&w, "implementation", "rc_filtered_pwm");
    jw_kv_u64(&w, "carrier_hz", PWM_TIMER_CLK_HZ / (TIM3_PWM_ARR + 1));
    jw_kv_u64(&w, "steps", TIM3_PWM_ARR + 1);
    jw_kv_bool(&w, "bipolar_as_two_unipolar_channels", true);
    jw_kv_str(&w, "note", "no on-chip DAC on this part -- needs an external RC low-pass per "
                          "channel (PIN_PWM_POS/PIN_PWM_NEG) plus a downstream differential-input stage");
    jw_obj_end(&w);

    jw_key(&w, "analog_position_servo"); jw_obj_begin(&w);
    jw_kv_str(&w, "controller", "analog-servo Control.c");
    jw_kv_str(&w, "source_sha256", AXIS_PID_SOURCE_SHA256);
    jw_kv_u64(&w, "control_tick_hz", 1000);
    jw_kv_u64(&w, "telemetry_stream_id", STREAM_ANALOG);
    jw_kv_str(&w, "telemetry_channels", "effort_code,reference_count; NOT ADC");
    jw_obj_end(&w);

    jw_kv_u64(&w, "encoder_count", NUM_ENCODERS);
    jw_key(&w, "encoder_decode_mults");
    jw_arr_begin(&w); jw_u64(&w, 4); jw_arr_end(&w);
    jw_kv_u64(&w, "encoder_counter_width_bits", 64);

    jw_kv_u64(&w, "analog_channels", NUM_ANALOG_CHANNELS);
    jw_kv_u64(&w, "analog_raw_bits", 12);
    jw_kv_bool(&w, "analog_simultaneous", true); /* ADC1+ADC2 regular-simultaneous dual mode, see axis_hw.cpp */
    jw_kv_u64(&w, "analog_min_period_ticks", ANALOG_MIN_PERIOD_TICKS);

    jw_key(&w, "memory");
    jw_obj_begin(&w);
    jw_kv_u64(&w, "max_command_samples", MAX_COMMAND_SAMPLES);
    /* Capture is unsolicited-push streamed, not buffered on-device: there is
     * no sample-count ceiling here. Real limits are host/USB drain rate and
     * run duration; see the DATA push cadence below and GET_RUN's per-stream
     * "frames_pushed"/sha256 for after-the-fact completeness verification. */
    jw_kv_str(&w, "capture_mode", "push_streamed");
    jw_kv_u64(&w, "push_chunk_samples", PUSH_CHUNK_SAMPLES);
    jw_obj_end(&w);

    jw_key(&w, "codecs");
    jw_arr_begin(&w); jw_str(&w, "raw_i64"); jw_str(&w, "delta_rle_i32"); jw_str(&w, "raw_i32"); jw_arr_end(&w);

    jw_kv_u64(&w, "service_tick_hz", TICK_HZ / TICKS_PER_SERVICE);
    jw_kv_u64(&w, "max_timing_error_ticks", TICKS_PER_SERVICE);

    jw_key(&w, "safety");
    jw_obj_begin(&w);
    jw_kv_bool(&w, "watchdog", true);
    jw_kv_bool(&w, "estop_input", true);
    jw_kv_bool(&w, "travel_limits", true);
    jw_kv_str(&w, "note", "USB CDC, CRC and this watchdog are not a safety-rated system");
    jw_obj_end(&w);

    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
    else send_error(seq, "HELLO", "INTERNAL", "response buffer overflow");
}

/* ---------------------------------- STATUS ---------------------------------- */

static const char *state_name(device_state_t s) {
    switch (s) {
        case ST_IDLE: return "IDLE";
        case ST_UPLOADING: return "UPLOADING";
        case ST_READY: return "READY";
        case ST_ARMED: return "ARMED";
        case ST_RUNNING: return "RUNNING";
        case ST_COMPLETE: return "COMPLETE";
        case ST_FAULT: return "FAULT";
    }
    return "IDLE";
}

static const char *fault_name(fault_reason_t r) {
    switch (r) {
        case FAULT_NONE: return "NONE";
        case FAULT_HOST_ABORT: return "HOST_ABORT";
        case FAULT_WATCHDOG: return "WATCHDOG";
        case FAULT_INTERLOCK: return "INTERLOCK";
        case FAULT_OVERFLOW: return "OVERFLOW";
        case FAULT_CAPTURE_TIMING: return "CAPTURE_TIMING";
        case FAULT_ESTOP: return "INTERLOCK";
    }
    return "NONE";
}

static void handle_STATUS(uint32_t seq) {
    fault_from_hw();
    axis_status_t st;
    axis_hw_get_status(&st);

    char buf[RESPONSE_BUF_LEN];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "STATUS");
    jw_kv_u64(&w, "boot_id", g_boot_id);
    jw_kv_str(&w, "state", state_name(g_state));
    if (g_have_run) jw_kv_str(&w, "run_id", g_run_id); else jw_kv_null(&w, "run_id");
    jw_kv_u64(&w, "tick", axis_hw_now_ticks());
    jw_kv_u64(&w, "executed_command_index", st.executed_command_index);

    jw_key(&w, "captured_sample_counts");
    jw_arr_begin(&w);
    for (uint32_t i = 0; i < NUM_CAPTURE_STREAMS; i++) jw_u64(&w, st.captured_count[i]);
    jw_arr_end(&w);

    jw_kv_bool(&w, "capture_complete", st.capture_complete);
    jw_kv_bool(&w, "output_enabled", st.output_enabled);
    if (g_state == ST_FAULT) {
        jw_kv_str(&w, "fault_reason", fault_name(st.fault_reason));
        jw_kv_u64(&w, "fault_tick", st.fault_tick);
    } else {
        jw_kv_null(&w, "fault_reason");
    }
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
    else send_error(seq, "STATUS", "INTERNAL", "response buffer overflow");
}

/* ---------------------------------- BEGIN ---------------------------------- */

static bool get_u64(JsonVariantConst obj, const char *key, uint64_t *out, uint64_t def, bool required) {
    JsonVariantConst v = obj[key];
    if (v.isNull()) {
        if (required) return false;
        *out = def;
        return true;
    }
    if (!v.is<long long>()) return false; /* rejects floats/strings: integers only */
    long long raw = v.as<long long>();
    if (raw < 0) return false;
    *out = (uint64_t)raw;
    return true;
}
static bool get_i64(JsonVariantConst obj, const char *key, int64_t *out, int64_t def, bool required) {
    JsonVariantConst v = obj[key];
    if (v.isNull()) {
        if (required) return false;
        *out = def;
        return true;
    }
    if (!v.is<long long>()) return false;
    *out = (int64_t)v.as<long long>();
    return true;
}
static bool get_bool(JsonVariantConst obj, const char *key, bool *out, bool def) {
    JsonVariantConst v = obj[key];
    if (v.isNull()) { *out = def; return true; }
    if (!v.is<bool>()) return false;
    *out = v.as<bool>();
    return true;
}

static void handle_BEGIN(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_IDLE) { send_error(seq, "BEGIN", "STATE", "device is not IDLE"); return; }

    const char *rid = doc["run_id"] | (const char *)nullptr;
    if (!rid || !rid[0] || strlen(rid) >= sizeof(g_run_id)) {
        send_error(seq, "BEGIN", "RANGE", "run_id missing or too long");
        return;
    }

    JsonVariantConst m = doc["manifest"];
    if (m.isNull() || !m.is<JsonObjectConst>()) {
        send_error(seq, "BEGIN", "RANGE", "manifest missing");
        return;
    }

    run_config_t rc;
    memset(&rc, 0, sizeof(rc));

    const char *drive = m["drive_interpretation"] | "position";
    if (strcmp(drive, "position") && strcmp(drive, "velocity") && strcmp(drive, "effort")) {
        send_error(seq, "BEGIN", "RANGE", "drive_interpretation must be position/velocity/effort");
        return;
    }

    const char *outmode = m["output_mode"] | (const char *)nullptr;
    if (!outmode) { send_error(seq, "BEGIN", "RANGE", "output_mode missing"); return; }
    if (!strcmp(outmode, "step_dir")) rc.output_mode = OUTPUT_STEP_DIR;
    else if (!strcmp(outmode, "quadrature_command")) rc.output_mode = OUTPUT_QUADRATURE_COMMAND;
    else if (!strcmp(outmode, "analog_effort")) rc.output_mode = OUTPUT_ANALOG_PWM_RC;
    else if (!strcmp(outmode, "analog_position_servo")) rc.output_mode = OUTPUT_ANALOG_POSITION_SERVO;
    else { send_error(seq, "BEGIN", "UNSUPPORTED", "unknown output_mode"); return; }
    if (rc.output_mode == OUTPUT_ANALOG_POSITION_SERVO) {
        if (strcmp(drive, "position")) { send_error(seq, "BEGIN", "RANGE", "servo requires position references"); return; }
        const char *names[] = {"servo_position_kp_q8", "servo_velocity_kp_q8", "servo_velocity_ki_q8",
            "servo_torque_limit_code", "servo_error_limit_counts", "servo_velocity_clamp"};
        int32_t *fields[] = {&rc.servo.position_kp_q8, &rc.servo.velocity_kp_q8, &rc.servo.velocity_ki_q8,
            &rc.servo.torque_limit_code, &rc.servo.error_limit_counts, &rc.servo.velocity_clamp};
        for (unsigned i=0; i<6; i++) {
            int64_t value;
            if (!get_i64(m, names[i], &value, 0, true) || value<0 || value>2147483647LL) {
                send_error(seq, "BEGIN", "RANGE", "invalid servo parameter"); return;
            }
            *fields[i]=(int32_t)value;
        }
    }

    uint64_t u;
    bool ok = true;
    ok &= get_u64(m, "command_sample_count", &u, 0, true); rc.command_sample_count = (uint32_t)u;
    ok &= get_u64(m, "command_period_ticks", &rc.command_period_ticks, 0, true);
    ok &= get_u64(m, "capture_pre_roll_ticks", &rc.capture_pre_roll_ticks, 0, false);
    ok &= get_u64(m, "capture_post_roll_ticks", &rc.capture_post_roll_ticks, 0, false);
    ok &= get_u64(m, "step_pulse_width_ticks", &u, TICKS_PER_SERVICE, false); rc.step_pulse_width_ticks = (uint32_t)u;
    ok &= get_u64(m, "dir_setup_ticks", &u, 0, false); rc.dir_setup_ticks = (uint32_t)u;
    ok &= get_u64(m, "dir_hold_ticks", &u, 0, false); rc.dir_hold_ticks = (uint32_t)u;
    ok &= get_bool(m, "output_idle_level_high", &rc.output_idle_level_high, false);
    ok &= get_bool(m, "enable_active_high", &rc.enable_active_high, true);
    ok &= get_bool(m, "output_enable_used", &rc.output_enable_used, true);
    ok &= get_bool(m, "quadrature_invert", &rc.quadrature_invert, false);
    int64_t fs;
    ok &= get_i64(m, "analog_output_full_scale", &fs, 32767, false); rc.analog_output_full_scale = (int32_t)fs;
    if (!ok) { send_error(seq, "BEGIN", "RANGE", "a numeric field was missing, non-integer, or the wrong type"); return; }

    JsonVariantConst encs = m["encoders"];
    if (!encs.isNull()) {
        if (!encs.is<JsonArrayConst>() || encs.as<JsonArrayConst>().size() > NUM_ENCODERS) {
            send_error(seq, "BEGIN", "RANGE", "too many encoders for this device (see HELLO.encoder_count)"); return;
        }
        uint32_t i = 0;
        for (JsonVariantConst e : encs.as<JsonArrayConst>()) {
            ok &= get_bool(e, "enabled", &rc.encoder_enabled[i], false);
            uint64_t dm;
            ok &= get_u64(e, "decode_mult", &dm, 4, false); rc.encoder_decode_mult[i] = (uint8_t)dm;
            ok &= get_bool(e, "invert", &rc.encoder_invert[i], false);
            ok &= get_bool(e, "travel_limit_enabled", &rc.travel_limit_enabled[i], false);
            int64_t tl;
            ok &= get_i64(e, "travel_limit_counts", &tl, 0, false); rc.travel_limit_counts[i] = (int32_t)tl;
            i++;
        }
    }
    ok &= get_u64(m, "encoder_sample_period_ticks", &rc.encoder_sample_period_ticks,
                  64ull * TICKS_PER_SERVICE, false);
    if (!ok) { send_error(seq, "BEGIN", "RANGE", "invalid encoder configuration"); return; }

    ok &= get_bool(m, "analog_enabled", &rc.analog_enabled, false);
    ok &= get_u64(m, "analog_sample_period_ticks", &rc.analog_sample_period_ticks,
                  200ull * TICKS_PER_SERVICE, false);
    ok &= get_u64(m, "watchdog_timeout_ticks", &rc.watchdog_timeout_ticks, 0, false);
    ok &= get_u64(m, "arming_timeout_ticks", &rc.arming_timeout_ticks, TICK_HZ * 10, false);
    if (!ok) { send_error(seq, "BEGIN", "RANGE", "invalid analog/watchdog/arming configuration"); return; }

    const char *conn = m["connection_policy"] | "supervised";
    if (!strcmp(conn, "supervised")) rc.connection_policy = CONN_SUPERVISED;
    else if (!strcmp(conn, "autonomous")) rc.connection_policy = CONN_AUTONOMOUS;
    else { send_error(seq, "BEGIN", "RANGE", "connection_policy must be supervised/autonomous"); return; }
    if (rc.connection_policy == CONN_SUPERVISED && rc.watchdog_timeout_ticks == 0) {
        send_error(seq, "BEGIN", "RANGE", "supervised connection_policy requires watchdog_timeout_ticks > 0");
        return;
    }

    const char *safe = m["safe_action"] | (const char *)nullptr;
    if (!safe) { send_error(seq, "BEGIN", "RANGE", "safe_action is required"); return; }
    if (!strcmp(safe, "disable_drive")) rc.safe_action = SAFE_DISABLE_DRIVE;
    else if (!strcmp(safe, "zero_output")) rc.safe_action = SAFE_ZERO_OUTPUT;
    else if (!strcmp(safe, "stop_pulses")) rc.safe_action = SAFE_STOP_PULSES;
    else { send_error(seq, "BEGIN", "UNSUPPORTED", "safe_action must be disable_drive/zero_output/stop_pulses"); return; }

    uint32_t seg_count = 0;
    segment_t segs[MAX_SEGMENTS];
    JsonVariantConst segv = m["segments"];
    if (!segv.isNull()) {
        if (!segv.is<JsonArrayConst>() || segv.as<JsonArrayConst>().size() > MAX_SEGMENTS) {
            send_error(seq, "BEGIN", "LIMIT", "too many segments for this device");
            return;
        }
        for (JsonVariantConst s : segv.as<JsonArrayConst>()) {
            const char *name = s["name"] | "segment";
            uint64_t a, b;
            if (!get_u64(s, "start", &a, 0, true) || !get_u64(s, "end", &b, 0, true)) {
                send_error(seq, "BEGIN", "RANGE", "segment start/end must be integers");
                return;
            }
            strncpy(segs[seg_count].name, name, sizeof(segs[seg_count].name) - 1);
            segs[seg_count].name[sizeof(segs[seg_count].name) - 1] = '\0';
            segs[seg_count].start = (uint32_t)a;
            segs[seg_count].end = (uint32_t)b;
            seg_count++;
        }
    }

    const char *hw_err = axis_hw_validate_config(&rc);
    if (hw_err) { send_error(seq, "BEGIN", hw_err, "rejected by hardware capability check"); return; }

    /* Accepted: commit state. */
    g_rc = rc;
    strncpy(g_run_id, rid, sizeof(g_run_id) - 1);
    g_run_id[sizeof(g_run_id) - 1] = '\0';
    g_have_run = true;
    strncpy(g_drive_interpretation, drive, sizeof(g_drive_interpretation) - 1);
    g_segment_count = seg_count;
    memcpy(g_segments, segs, sizeof(segment_t) * seg_count);

    cmdbuf_reset(cmdbuf_instance(), g_rc.command_sample_count);
    for (uint32_t i = 0; i < NUM_ENCODERS; i++) {
        capstream_reset(capstream_instance(i), g_rc.encoder_sample_period_ticks, 1, true /* widen to i64 */);
    }
    capstream_reset(capstream_instance(STREAM_ANALOG), g_rc.analog_sample_period_ticks,
                     NUM_ANALOG_CHANNELS, false /* raw_i32 */);
    g_flushed_this_run = false;

    axis_hw_configure(&g_rc);

    /* Do not put a 2KB canonical buffer on the stack while the parsed
     * request still occupies the heap: they collide on this 20KB MCU. */
    uint8_t digest[32];
    sha256_ctx ctx; sha256_init(&ctx);
    build_canonical_manifest(&ctx);
    sha256_final(&ctx, digest);
    sha256_hex(digest, g_manifest_hash_hex);

    g_state = ST_UPLOADING;

    char buf[256];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "BEGIN");
    jw_kv_str(&w, "manifest_hash", g_manifest_hash_hex);
    jw_obj_end(&w);
    size_t rn = jw_finish(&w);
    if (rn) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)rn);
}

/* ------------------------------- PUT_COMMAND ------------------------------- */

static void handle_PUT_COMMAND(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_UPLOADING) { send_error(seq, "PUT_COMMAND", "STATE", "device is not UPLOADING"); return; }
    if (!check_run_id(doc, seq, "PUT_COMMAND")) return;

    uint64_t off64;
    if (!get_u64(doc, "offset", &off64, 0, true) || off64 > MAX_COMMAND_SAMPLES) {
        send_error(seq, "PUT_COMMAND", "RANGE", "offset missing, invalid, or out of range");
        return;
    }

    JsonVariantConst values = doc["values"];
    if (values.isNull() || !values.is<JsonArrayConst>()) {
        send_error(seq, "PUT_COMMAND", "RANGE", "values must be an array of decimal strings");
        return;
    }
    JsonArrayConst arr = values.as<JsonArrayConst>();
    if (arr.size() == 0) { send_error(seq, "PUT_COMMAND", "RANGE", "values must not be empty"); return; }

    /* Parsed and stored one sample at a time -- a payload-sized scratch
     * array isn't needed, and cmdbuf_put's offset/range/conflict checks
     * apply identically per sample. */
    uint32_t i = 0;
    for (JsonVariantConst v : arr) {
        const char *s = v.as<const char *>();
        int64_t sample;
        if (!s || !parse_i64_strict(s, &sample)) {
            send_error(seq, "PUT_COMMAND", "RANGE", "values must be signed decimal strings");
            return;
        }
        const char *err = nullptr;
        if (!cmdbuf_put(cmdbuf_instance(), (uint32_t)off64 + i, &sample, 1, &err)) {
            send_error(seq, "PUT_COMMAND", err ? err : "RANGE", nullptr);
            return;
        }
        i++;
    }

    char buf[128];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "PUT_COMMAND");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* ----------------------------------- SEAL ----------------------------------- */

static void handle_SEAL(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_UPLOADING) { send_error(seq, "SEAL", "STATE", "device is not UPLOADING"); return; }
    if (!check_run_id(doc, seq, "SEAL")) return;

    const char *given_hash = doc["sha256"] | (const char *)nullptr;
    if (!given_hash) { send_error(seq, "SEAL", "RANGE", "sha256 missing"); return; }

    command_buffer_t *cb = cmdbuf_instance();
    if (!cmdbuf_is_complete(cb)) { send_error(seq, "SEAL", "INCOMPLETE", "not all command samples were uploaded"); return; }

    uint8_t digest[32];
    cmdbuf_hash_canonical(cb, digest);
    sha256_hex(digest, g_command_hash_hex);
    bool match = true;
    for (int i = 0; given_hash[i] || g_command_hash_hex[i]; i++) {
        char a = given_hash[i], b = g_command_hash_hex[i];
        if (a >= 'A' && a <= 'F') a += 32;
        if (a != b) { match = false; break; }
    }
    if (!match) { send_error(seq, "SEAL", "HASH", "uploaded command data does not match the declared sha256"); return; }

    /* Edge-rate / timing feasibility: the runtime step generator assumes
     * at most one emitted edge per service tick window (see axis_hw.cpp),
     * and that pulse width fits within that same window twice over.
     * Doesn't apply to analog_effort: there are no edges, just a held
     * code per interval (zero-order hold). */
    if (g_rc.output_mode == OUTPUT_ANALOG_POSITION_SERVO) {
        for (uint32_t i=0; i<g_rc.command_sample_count; i++) {
            int32_t v=cmdbuf_value_at(cb,i);
            if (v>4096 || v<-4096 || ((i==0 || i+1==g_rc.command_sample_count) && v!=0)) {
                send_error(seq, "SEAL", "RANGE", "servo references need zero endpoints and magnitude <=4096 counts"); return;
            }
        }
    }
    if (!axis_output_is_pwm(g_rc.output_mode)) {
        uint32_t min_ticks_per_edge = 2 * TICKS_PER_SERVICE;
        if (g_rc.output_mode == OUTPUT_STEP_DIR && 2 * g_rc.step_pulse_width_ticks > min_ticks_per_edge) {
            min_ticks_per_edge = 2 * g_rc.step_pulse_width_ticks;
        }
        int32_t prev = 0;
        for (uint32_t i = 0; i < g_rc.command_sample_count; i++) {
            int32_t cur = cmdbuf_value_at(cb, i);
            int32_t n = cur - prev;
            uint32_t n_abs = (uint32_t)(n < 0 ? -n : n);
            if (n_abs > 0 && g_rc.command_period_ticks / n_abs < min_ticks_per_edge) {
                send_error(seq, "SEAL", "LIMIT", "a command interval requires an edge rate this device cannot schedule");
                return;
            }
            prev = cur;
        }
    }

    g_state = ST_READY;
    char buf[128];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "SEAL");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* ----------------------------------- ARM ----------------------------------- */

static void handle_ARM(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_READY) { send_error(seq, "ARM", "STATE", "device is not READY"); return; }
    if (!check_run_id(doc, seq, "ARM")) return;
    if (axis_hw_estop_asserted()) { send_error(seq, "ARM", "INTERLOCK", "ESTOP is asserted"); return; }

    axis_hw_latch_origin();
    axis_hw_feed_watchdog();
    g_arm_deadline = axis_hw_now_ticks() + g_rc.arming_timeout_ticks;
    g_state = ST_ARMED;

    char buf[128];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "ARM");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* ---------------------------------- START ---------------------------------- */

static void handle_START(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_ARMED) { send_error(seq, "START", "STATE", "device is not ARMED"); return; }
    if (!check_run_id(doc, seq, "START")) return;
    if (axis_hw_now_ticks() > g_arm_deadline) {
        g_state = ST_READY;
        send_error(seq, "START", "STATE", "arming expired; re-ARM");
        return;
    }

    uint64_t now = axis_hw_now_ticks();
    uint64_t command_start = now + g_rc.capture_pre_roll_ticks + SCHEDULE_MARGIN_TICKS;
    uint64_t capture_start = command_start - g_rc.capture_pre_roll_ticks;

    axis_hw_feed_watchdog();
    axis_hw_schedule_start(command_start, capture_start);
    g_state = ST_RUNNING;

    char buf[160];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "START");
    jw_kv_u64(&w, "command_start_tick", command_start);
    jw_kv_u64(&w, "capture_start_tick", capture_start);
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* -------------------------------- KEEPALIVE --------------------------------- */

static void handle_KEEPALIVE(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_ARMED && g_state != ST_RUNNING) {
        send_error(seq, "KEEPALIVE", "STATE", "device is not ARMED or RUNNING");
        return;
    }
    if (!check_run_id(doc, seq, "KEEPALIVE")) return;
    axis_hw_feed_watchdog();

    char buf[96];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "KEEPALIVE");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* ----------------------------------- ABORT ----------------------------------- */

static void handle_ABORT(uint32_t seq, JsonVariantConst doc) {
    if (g_state != ST_ARMED && g_state != ST_RUNNING && g_state != ST_UPLOADING && g_state != ST_READY) {
        send_error(seq, "ABORT", "STATE", "no active run to abort");
        return;
    }
    if (g_have_run && !check_run_id(doc, seq, "ABORT")) return;

    axis_hw_abort(FAULT_HOST_ABORT);
    /* Preserve the RUNNING -> FAULT transition so the final partial
     * capture chunks are flushed, just as for local watchdog/overflow. */
    fault_from_hw();
    g_state = ST_FAULT;

    char buf[96];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "ABORT");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* ---------------------------------- GET_RUN ---------------------------------- */

static void handle_GET_RUN(uint32_t seq, JsonVariantConst doc) {
    fault_from_hw();
    if (g_state != ST_COMPLETE && g_state != ST_FAULT) {
        send_error(seq, "GET_RUN", "STATE", "run data is only available after COMPLETE or FAULT");
        return;
    }
    if (!check_run_id(doc, seq, "GET_RUN")) return;

    axis_status_t st;
    axis_hw_get_status(&st);

    char buf[RESPONSE_BUF_LEN];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "GET_RUN");
    jw_kv_str(&w, "run_id", g_run_id);
    jw_kv_str(&w, "manifest_hash", g_manifest_hash_hex);
    jw_kv_str(&w, "command_hash", g_command_hash_hex);
    jw_kv_u64(&w, "command_start_tick", st.command_start_tick);
    jw_kv_u64(&w, "capture_start_tick", st.capture_start_tick);
    jw_kv_u64(&w, "run_end_tick", st.run_end_tick);
    jw_kv_bool(&w, "capture_complete", st.capture_complete);
    if (st.fault) { jw_kv_str(&w, "fault_reason", fault_name(st.fault_reason)); jw_kv_u64(&w, "fault_tick", st.fault_tick); }
    else jw_kv_null(&w, "fault_reason");

    jw_key(&w, "streams");
    jw_arr_begin(&w);
    for (uint32_t s = 0; s < NUM_CAPTURE_STREAMS; s++) {
        bool enabled = axis_stream_enabled(&g_rc, s);
        if (!enabled) continue;
        capture_stream_t *cs = capstream_instance(s);
        jw_obj_begin(&w);
        jw_kv_u64(&w, "stream_id", s);
        jw_kv_str(&w, "role", (s < NUM_ENCODERS) ? "encoder" :
            (g_rc.output_mode == OUTPUT_ANALOG_POSITION_SERVO ? "servo_effort_reference" : "analog"));
        jw_kv_u64(&w, "channels", cs->channels);
        jw_kv_u64(&w, "period_ticks", cs->period_ticks);
        jw_kv_u64(&w, "first_sample_tick", cs->first_sample_tick);
        jw_kv_u64(&w, "sample_count", cs->count);
        /* Total DATA frames actually pushed for this stream this run --
         * compare against the host's own received-frame count as a cheap
         * completeness check, on top of the per-frame push sequence. */
        jw_kv_u64(&w, "frames_pushed", cs->next_push_seq);
        jw_key(&w, "codecs");
        jw_arr_begin(&w);
        if (s < NUM_ENCODERS) { jw_str(&w, "raw_i64"); jw_str(&w, "delta_rle_i32"); }
        else jw_str(&w, "raw_i32");
        jw_arr_end(&w);
        uint8_t digest[32]; char hex[65];
        capstream_final_hash(cs, digest);
        sha256_hex(digest, hex);
        jw_kv_str(&w, "sha256", hex);
        jw_obj_end(&w);
    }
    jw_arr_end(&w);

    jw_key(&w, "segments");
    jw_arr_begin(&w);
    for (uint32_t i = 0; i < g_segment_count; i++) {
        jw_obj_begin(&w);
        jw_kv_str(&w, "name", g_segments[i].name);
        jw_kv_u64(&w, "start", g_segments[i].start);
        jw_kv_u64(&w, "end", g_segments[i].end);
        jw_obj_end(&w);
    }
    jw_arr_end(&w);

    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
    else send_error(seq, "GET_RUN", "INTERNAL", "response buffer overflow");
}

/* ------------------------------ capture push ------------------------------ */

/* Builds and sends one unsolicited DATA frame from whatever capstream_drain
 * finds pending for this stream, if anything. DATA's `block_index` field is
 * repurposed as this stream's own push-sequence counter (see run_buffer.h);
 * `sequence`=0 marks the frame itself as unsolicited, per the wire doc's
 * "0 reserved for unsolicited events". A no-op if nothing is pending. */
static void push_stream_data(uint32_t stream_id, capture_stream_t *cs, uint8_t codec) {
    static uint8_t payload[24 + (PUSH_CHUNK_SAMPLES * NUM_ANALOG_CHANNELS * 8)];
    uint32_t sample_count, seq, body_len;
    uint64_t first_tick;
    if (!capstream_drain(cs, codec, payload + 24, sizeof(payload) - 24,
                          &sample_count, &first_tick, &seq, &body_len)) {
        return;
    }

    payload[0] = (uint8_t)stream_id; payload[1] = (uint8_t)(stream_id >> 8);
    payload[2] = (uint8_t)(stream_id >> 16); payload[3] = (uint8_t)(stream_id >> 24);
    payload[4] = (uint8_t)seq; payload[5] = (uint8_t)(seq >> 8);
    payload[6] = (uint8_t)(seq >> 16); payload[7] = (uint8_t)(seq >> 24);
    for (int b = 0; b < 8; b++) payload[8 + b] = (uint8_t)(first_tick >> (8 * b));
    payload[16] = (uint8_t)sample_count; payload[17] = (uint8_t)(sample_count >> 8);
    payload[18] = (uint8_t)(sample_count >> 16); payload[19] = (uint8_t)(sample_count >> 24);
    payload[20] = codec;
    payload[21] = payload[22] = payload[23] = 0;

    send_frame(4 /* DATA */, 0 /* unsolicited */, payload, 24 + body_len);
}

/* Drains and pushes every stream with a pending chunk. Each stream can have
 * at most one chunk pending under normal operation (capstream_append faults
 * the run before a third chunk's worth could ever queue up), so draining
 * twice per stream is enough to catch both halves without an unbounded loop. */
static void capture_streams_service(void) {
    if (!g_have_run) return;
    for (uint32_t s = 0; s < NUM_CAPTURE_STREAMS; s++) {
        bool enabled = axis_stream_enabled(&g_rc, s);
        if (!enabled) continue;
        uint8_t codec = (s < NUM_ENCODERS) ? 1 /* raw_i64 */ : 3 /* raw_i32 */;
        capture_stream_t *cs = capstream_instance(s);
        push_stream_data(s, cs, codec);
        push_stream_data(s, cs, codec);
    }
}

/* ----------------------------------- CLEAR ----------------------------------- */

static void handle_CLEAR(uint32_t seq, JsonVariantConst doc) {
    fault_from_hw();
    if (g_state != ST_COMPLETE && g_state != ST_FAULT) {
        send_error(seq, "CLEAR", "STATE", "device is not COMPLETE or FAULT");
        return;
    }
    if (!check_run_id(doc, seq, "CLEAR")) return;

    axis_hw_clear_run();
    g_have_run = false;
    g_state = ST_IDLE;

    char buf[96];
    json_writer_t w;
    jw_init(&w, buf, sizeof(buf));
    jw_obj_begin(&w);
    jw_kv_bool(&w, "ok", true);
    jw_kv_str(&w, "op", "CLEAR");
    jw_obj_end(&w);
    size_t n = jw_finish(&w);
    if (n) send_frame(2, seq, (const uint8_t *)buf, (uint32_t)n);
}

/* --------------------------------- dispatch ---------------------------------- */

static void handle_request_json(uint32_t seq, const char *json, size_t len) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, json, len);
    if (err) { send_error(seq, "?", "INTERNAL", "malformed JSON request"); return; }

    const char *op = doc["op"] | (const char *)nullptr;
    if (!op) { send_error(seq, "?", "INTERNAL", "missing op"); return; }

    fault_from_hw();

    if (!strcmp(op, "HELLO")) handle_HELLO(seq);
    else if (!strcmp(op, "STATUS")) handle_STATUS(seq);
    else if (!strcmp(op, "BEGIN")) handle_BEGIN(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "PUT_COMMAND")) handle_PUT_COMMAND(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "SEAL")) handle_SEAL(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "ARM")) handle_ARM(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "START")) handle_START(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "KEEPALIVE")) handle_KEEPALIVE(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "ABORT")) handle_ABORT(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "GET_RUN")) handle_GET_RUN(seq, doc.as<JsonVariantConst>());
    else if (!strcmp(op, "CLEAR")) handle_CLEAR(seq, doc.as<JsonVariantConst>());
    else send_error(seq, op, "UNSUPPORTED", "unknown op");
}

/* ----------------------------------- public ----------------------------------- */

void protocol_init(void) {
    g_rx_len = 0;
    g_state = ST_IDLE;
    g_have_run = false;

    const uint32_t *uid = (const uint32_t *)0x1FFFF7E8u;
    g_boot_id = (uint32_t)micros() ^ uid[0] ^ (uid[1] << 1) ^ (uid[2] << 2);
    snprintf(g_device_serial, sizeof(g_device_serial), "%08lX%08lX%08lX",
             (unsigned long)uid[0], (unsigned long)uid[1], (unsigned long)uid[2]);

    axis_hw_init();
}

void protocol_poll(void) {
    fault_from_hw();
    if (g_state == ST_ARMED && axis_hw_now_ticks() > g_arm_deadline) {
        axis_hw_clear_run();
        g_state = ST_READY;
    }
    capture_streams_service();
}
