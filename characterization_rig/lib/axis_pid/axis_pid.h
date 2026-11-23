#ifndef AXIS_PID_H
#define AXIS_PID_H
#include <stdint.h>
#include <stdbool.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Shared by MCU and native backend. Effort is code/32767, NOT current.
 * The tuning search lives on the PC; no online identification/dither here. */
typedef struct {
    int32_t position_kp_q8, velocity_kp_q8, velocity_ki_q8;
    int32_t torque_limit_code, error_limit_counts, velocity_clamp;
} axis_pid_config_t;
bool axis_pid_validate(const axis_pid_config_t *cfg);
bool axis_pid_init(const axis_pid_config_t *cfg, int32_t position);
/* 1 kHz ONLY. Latches failure until init; every failed call returns zero. */
bool axis_pid_tick(int32_t reference, int32_t position, int32_t *effort);
#ifdef __cplusplus
}
#endif
#endif
