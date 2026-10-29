#ifndef WHEEL_ENCODER_H
#define WHEEL_ENCODER_H
#include <stdint.h>

/* Published together every speed window: counts, counts/s, milliseconds.
 * valid describes timing, not physical wiring. position_valid latches low
 * after a lost window; stopped Mode2 Start or board reset rebases it. */
extern volatile uint32_t wheel_encoder_raw, wheel_encoder_seq;
extern volatile int32_t wheel_encoder_delta, wheel_position_count;
extern volatile float wheel_speed_cps, wheel_encoder_dt_ms;
extern volatile uint8_t wheel_encoder_valid, wheel_position_valid;
extern volatile uint32_t wheel_encoder_errors;
typedef struct {
    uint32_t sequence,age_ms;
    uint32_t position_epoch;
    int32_t position_count;
    float speed_cps,dt_ms;
    uint8_t valid,position_valid;
} wheel_encoder_sample_t;
void wheel_encoder_snapshot(wheel_encoder_sample_t *sample);
uint8_t wheel_encoder_rebase_position(void); /* Stopped Mode2 Start only; no motor writes. */
void wheel_encoder_init(void); /* After encoder_dir_init; boot only. */
void wheel_encoder_tick_1ms(void); /* Always-on timer; observation only. */
#endif
