#include "Estimation/Attitude/attitude.h"
#include "HW_Drivers/IMU/imu.h"
#include "Estimation/Attitude/r10h/include/r10h_attitude.h"
#include "Estimation/Attitude/r10h/core/candidate/compare_core.h"
#include "IfxStm.h"
#include <math.h>
#include <string.h>

#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif
attitude_t att;
att_diag_t att_diag;
extern compare_result_t v2_compare_result;
static ahrs_output_t pending;
static uint8 initialized, fault, has_pending, clock_started;
static uint16 accepted_ticks;
static uint32 last_tick;
static float clock_us;
#define ATT_READY_SAMPLES (3000u) /* Preserve the previous 3 s start interlock. */

static uint32_t clock_ticks(void) { return IfxStm_getLower(&MODULE_STM0); }
static ahrs_sample_t sample_from_imu(const imu_data_t *s)
{
    ahrs_sample_t a;
    a.gx=s->gyro_x; a.gy=s->gyro_y; a.gz=s->gyro_z;
    a.ax=s->acc_x; a.ay=s->acc_y; a.az=s->acc_z;
    return a;
}
static uint8 valid_output(const ahrs_output_t *o)
{
    return (uint8)(!o->fault && isfinite(o->roll) && isfinite(o->pitch) &&
        isfinite(o->yaw) && isfinite(o->acc_weight) && isfinite(o->bias_z_dps));
}
static float wrap180(float angle)
{
    float wrapped=fmodf(angle,360.0f);
    if (wrapped>180.0f) wrapped-=360.0f;
    if (wrapped < -180.0f) wrapped+=360.0f;
    return wrapped;
}

void attitude_init(void)
{
    imu_data_t mean;
    ahrs_sample_t initial;
    memset(&att,0,sizeof(att)); memset(&att_diag,0,sizeof(att_diag));
    initialized=fault=has_pending=clock_started=0; accepted_ticks=0;
    clock_us=1000000.0f/IfxStm_getFrequency(&MODULE_STM0);
    if (!isfinite(clock_us) || clock_us<=0.0f || !imu_get_startup_mean(&mean))
    { fault=1; return; }
    initial=sample_from_imu(&mean);
    initialized=r10h_init(&initial);
    if (!initialized) { fault=1; return; }
    r10h_set_clock(clock_ticks,clock_us);
    /* Core aligned to measured gravity. Do not run a synthetic update,
     * subtract startup bias again, negate acceleration, or filter outside. */
    att.roll=atan2f(mean.acc_y,mean.acc_z)*AHRS_RAD_TO_DEG;
    att.pitch=atan2f(-mean.acc_x,sqrtf(mean.acc_y*mean.acc_y+mean.acc_z*mean.acc_z))*AHRS_RAD_TO_DEG;
    imu_publish_attitude_yaw(0.0f,0.0f);
}
uint8 attitude_converged(void)
{
    return (uint8)(initialized && !fault && !imu_link_lost() && accepted_ticks>=ATT_READY_SAMPLES);
}
uint8 attitude_diverged(void) { return fault; }
void attitude_update_rate(void)
{
    att.roll_rate=imu.gyro_x; att.pitch_rate=imu.gyro_y; att.yaw_rate=imu.gyro_z;
}

/* Exactly one CPU0 call per 1 ms sensor sample. Missed/early ticks latch a
 * fault; never catch up, integrate stale input, or auto-reset moving states. */
void attitude_tick1ms(void)
{
    imu_data_t raw;
    ahrs_sample_t input;
    uint32 now=clock_ticks();
    float elapsed;
    if (!initialized || fault) return;
    if (clock_started)
    {
        att_diag.period_us=(float)(now-last_tick)*clock_us;
        if (att_diag.period_us<750.0f || att_diag.period_us>1250.0f)
        { fault=1; has_pending=0; return; }
    }
    clock_started=1; last_tick=now;
    if (!imu_get_raw_sample(&raw)) { fault=1; has_pending=0; return; }
    input=sample_from_imu(&raw);
    if (!r10h_update(&input) || !r10h_get_output(&pending) ||
        !valid_output(&pending) || !valid_output(&v2_compare_result.b) ||
        !valid_output(&v2_compare_result.c))
    { fault=1; has_pending=0; return; }
    elapsed=(float)(clock_ticks()-now)*clock_us;
    att_diag.update_us=elapsed;
    if (elapsed>att_diag.max_update_us) att_diag.max_update_us=elapsed;
    if (elapsed>=1000.0f) { fault=1; has_pending=0; return; }
    att_diag.acc_norm=sqrtf(raw.acc_x*raw.acc_x+raw.acc_y*raw.acc_y+raw.acc_z*raw.acc_z);
    att_diag.acc_weight=pending.acc_weight;
    att_diag.acc_valid=(uint8)(pending.acc_weight>0.0f);
    att_diag.bias_z=pending.bias_z_dps*AHRS_DEG_TO_RAD; /* Diagnostic retains rad/s. */
    att_diag.is_static=pending.is_static; /* Never a motor permission. */
    if (accepted_ticks<ATT_READY_SAMPLES) ++accepted_ticks;
    has_pending=1;
}

/* 1 ms publication precedes all angle-loop calculations. CPU1 feedback
 * retains its independent 5 ms schedule; the estimator still runs at 1 kHz. */
void attitude_update(void)
{
    float yaw_delta;
    if (!has_pending || fault || imu_link_lost()) return;
    yaw_delta=pending.yaw-att.yaw;
    att.roll=pending.roll; att.pitch=pending.pitch; att.yaw=pending.yaw;
    att.yaw_wrapped=wrap180(att.yaw);
    imu_publish_attitude_yaw(att.yaw,yaw_delta);
    has_pending=0;
}
#if defined(__TASKING__)
#pragma section all restore
#endif
