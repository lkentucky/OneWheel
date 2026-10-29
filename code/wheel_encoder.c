/* Ground-wheel observation. No motor commands or balance-controller writes. */
#include "zf_common_headfile.h"
#include "wheel_encoder.h"
#include "IfxStm.h"

volatile uint32_t wheel_encoder_raw, wheel_encoder_seq;
volatile int32_t wheel_encoder_delta, wheel_position_count;
volatile float wheel_speed_cps, wheel_encoder_dt_ms;
volatile uint8_t wheel_encoder_valid, wheel_position_valid;
volatile uint32_t wheel_encoder_errors;

static uint8_t initialized, position_continuous;
static uint16_t previous_count;
static uint32_t previous_ticks, window_ticks, read_ticks, speed_ticks, gap_ticks;
static uint32_t published_ticks;
static uint32_t position_epoch;
static float clock_hz;
static int32_t position_count, window_count;

static void publish(uint16_t raw, uint32_t elapsed, uint8_t valid)
{
    published_ticks=window_ticks+elapsed;
    wheel_encoder_raw=raw;
    wheel_encoder_delta=valid ? window_count : 0;
    wheel_speed_cps=valid ? (float)window_count*clock_hz/(float)elapsed : 0.0f;
    wheel_encoder_dt_ms=(float)elapsed*1000.0f/clock_hz;
    wheel_position_count=position_count;
    wheel_encoder_valid=valid;
    wheel_position_valid=position_continuous;
    ++wheel_encoder_seq;
}

void wheel_encoder_init(void)
{
    uint32_t irq=interrupt_global_disable();
    initialized=0;
    wheel_encoder_raw=wheel_encoder_seq=wheel_encoder_errors=0;
    wheel_encoder_delta=wheel_position_count=0;
    wheel_speed_cps=wheel_encoder_dt_ms=0;
    wheel_encoder_valid=wheel_position_valid=0;
    position_count=window_count=0;
    clock_hz=IfxStm_getFrequency(&MODULE_STM0);
    if (!(clock_hz>=1000.0f && clock_hz<=1000000000.0f)) {
        wheel_encoder_errors=1;
        interrupt_global_enable(irq);
        return;
    }
    read_ticks=(uint32_t)(clock_hz*(WHEEL_ENCODER_READ_MS*0.001f));
    speed_ticks=(uint32_t)(clock_hz*(WHEEL_ENCODER_SPEED_MS*0.001f));
    gap_ticks=(uint32_t)(clock_hz*(WHEEL_ENCODER_MAX_GAP_MS*0.001f));
    previous_count=(uint16_t)encoder_get_count(MOTOR_ENCODER_TIM);
    previous_ticks=window_ticks=IfxStm_getLower(&MODULE_STM0);
    published_ticks=previous_ticks;
    ++position_epoch;
    wheel_encoder_raw=previous_count;
    position_continuous=1;
    initialized=1; /* The key timer may already be running. */
    interrupt_global_enable(irq);
}

uint8_t wheel_encoder_rebase_position(void)
{
    uint32_t irq=interrupt_global_disable();
    if (!initialized) {interrupt_global_enable(irq);return 0;}
    /* A previous lost window is irrelevant to a new stopped control session.
     * Preserve the hardware counter and error history; start a complete window. */
    previous_count=(uint16_t)encoder_get_count(MOTOR_ENCODER_TIM);
    previous_ticks=window_ticks=published_ticks=IfxStm_getLower(&MODULE_STM0);
    position_count=window_count=0;position_continuous=1;++position_epoch;
    wheel_encoder_raw=previous_count;wheel_encoder_delta=wheel_position_count=0;
    wheel_speed_cps=wheel_encoder_dt_ms=0;
    wheel_encoder_valid=wheel_position_valid=0;
    interrupt_global_enable(irq);
    return 1;
}

void wheel_encoder_snapshot(wheel_encoder_sample_t *sample)
{
    uint32_t now,irq;
    if (!sample) return;
    irq=interrupt_global_disable();
    sample->sequence=wheel_encoder_seq;sample->speed_cps=wheel_speed_cps;
    sample->dt_ms=wheel_encoder_dt_ms;sample->valid=wheel_encoder_valid;
    sample->position_count=wheel_position_count;sample->position_valid=wheel_position_valid;
    sample->position_epoch=position_epoch;
    now=IfxStm_getLower(&MODULE_STM0);
    sample->age_ms=initialized?(uint32_t)((float)(uint32_t)(now-published_ticks)*1000.0f/clock_hz):UINT32_MAX;
    interrupt_global_enable(irq);
}

void wheel_encoder_tick_1ms(void)
{
    uint32_t now, elapsed, window_elapsed;
    uint16_t raw, modular_delta;
    int32_t delta;
    if (!initialized) return;
    now=IfxStm_getLower(&MODULE_STM0);
    elapsed=(uint32_t)(now-previous_ticks);
    if (elapsed<read_ticks) return;
    raw=(uint16_t)encoder_get_count(MOTOR_ENCODER_TIM);
    modular_delta=(uint16_t)(raw-previous_count);
    previous_count=raw;previous_ticks=now;
    window_elapsed=(uint32_t)(now-window_ticks);
    /* Long gaps and exactly half a hardware range have ambiguous displacement. */
    if (elapsed>gap_ticks || modular_delta==32768u) {
        ++wheel_encoder_errors;position_continuous=0;
        publish(raw,window_elapsed,0);
        window_count=0;window_ticks=now;
        return;
    }
    delta=modular_delta<32768u ? (int32_t)modular_delta : (int32_t)modular_delta-65536;
    delta*=WHEEL_ENCODER_SIGN;
    /* Keep diagnostic positions exact in float32 telemetry; avoid overflow. */
    if (position_count+delta>WHEEL_ENCODER_POSITION_LIMIT ||
        position_count+delta< -WHEEL_ENCODER_POSITION_LIMIT) {
        ++wheel_encoder_errors;position_continuous=0;
        wheel_position_valid=0; /* Invalidate immediately, before the next published window. */
    } else position_count+=delta;
    window_count+=delta;
    if (window_elapsed>=speed_ticks) {
        publish(raw,window_elapsed,1);
        window_count=0;window_ticks=now;
    }
}
