#include "wheel_hold.h"
#include "config.h"
#include "pid.h"
#include <math.h>

volatile uint8_t wheel_hold_enabled, wheel_feedback_valid;
volatile int32_t wheel_hold_state;
volatile uint32_t wheel_feedback_age_ms;
volatile float wheel_target_cps, wheel_speed_error, wheel_tilt_offset, wheel_speed_integral;
float wheel_speed_kp=WHEEL_SPEED_KP, wheel_speed_ki=WHEEL_SPEED_KI;
float wheel_tilt_limit=WHEEL_SPEED_TILT_LIMIT_DEG, wheel_i_limit=WHEEL_SPEED_I_LIMIT_DEG;
volatile int32_t wheel_ref_count,wheel_pos_error;
float wheel_pos_kp=WHEEL_POSITION_KP,wheel_speed_limit=WHEEL_POSITION_SPEED_LIMIT_CPS;
static pid_controller_t speed_pid;
static pid_controller_t position_pid;
static float position_elapsed_ms;
static uint32_t reference_epoch;
static uint8_t have_reference;
static uint32_t last_sequence;
static uint8_t have_frame;
static int fault_reason;

void wheel_hold_suspend(void)
{
    pid_reset(&speed_pid);
    pid_reset(&position_pid);position_elapsed_ms=0;
    last_sequence=0;have_frame=0;
    wheel_feedback_valid=0;
    wheel_target_cps=wheel_speed_error=wheel_tilt_offset=wheel_speed_integral=0.0f;
    if (wheel_hold_state>=0) wheel_hold_state=1;
}

void wheel_hold_reset(void)
{
    wheel_hold_suspend();
    wheel_hold_state=0;fault_reason=0;
    have_reference=0;reference_epoch=0;wheel_ref_count=wheel_pos_error=0;
    wheel_feedback_valid=0;wheel_feedback_age_ms=UINT32_MAX;
}

void wheel_hold_defaults(void)
{
    wheel_speed_kp=WHEEL_SPEED_KP;wheel_speed_ki=WHEEL_SPEED_KI;
    wheel_tilt_limit=WHEEL_SPEED_TILT_LIMIT_DEG;wheel_i_limit=WHEEL_SPEED_I_LIMIT_DEG;
    wheel_pos_kp=WHEEL_POSITION_KP;wheel_speed_limit=WHEEL_POSITION_SPEED_LIMIT_CPS;
    wheel_hold_reset();
}

static int fail(int reason)
{
    wheel_hold_suspend();
    fault_reason=reason;wheel_hold_state=reason==9?-2:-1;
    wheel_feedback_valid=0;
    return -reason;
}

int wheel_hold_update(const wheel_encoder_sample_t *sample, int freeze_integral)
{
    float output;
    int captured_reference=0;
    /* Runs only on the CPU0 control ISR. Menu/UDP writes run on the same CPU's
     * main loop and cannot interrupt these reads; the encoder has a snapshot. */
    if (fault_reason) return -fault_reason;
    if (!isfinite(wheel_speed_kp) || wheel_speed_kp<0.0f || wheel_speed_kp>WHEEL_SPEED_GAIN_MAX ||
        !isfinite(wheel_speed_ki) || wheel_speed_ki<0.0f || wheel_speed_ki>WHEEL_SPEED_GAIN_MAX ||
        !isfinite(wheel_tilt_limit) || wheel_tilt_limit<0.0f || wheel_tilt_limit>WHEEL_SPEED_TILT_MAX_DEG ||
        !isfinite(wheel_i_limit) || wheel_i_limit<0.0f || wheel_i_limit>WHEEL_SPEED_TILT_MAX_DEG ||
        !isfinite(wheel_pos_kp) || wheel_pos_kp<0.0f || wheel_pos_kp>WHEEL_POSITION_GAIN_MAX ||
        !isfinite(wheel_speed_limit) || wheel_speed_limit<0.0f || wheel_speed_limit>WHEEL_POSITION_SPEED_MAX_CPS ||
        (WHEEL_SPEED_TILT_SIGN!=1.0f && WHEEL_SPEED_TILT_SIGN!=-1.0f)) return fail(9);
    if (!sample) return fail(7);
    wheel_feedback_age_ms=sample->age_ms;
    if (sample->age_ms>=WHEEL_SPEED_ENCODER_TIMEOUT_MS) return fail(8);
    if (have_reference && sample->position_epoch!=reference_epoch) return fail(10);
    /* Sequence 0 is also a legitimate uint32 wrap; only the initial empty
     * snapshot has both valid=0 and dt=0. A published bad window must fault. */
    if (!have_reference && !sample->valid && sample->dt_ms==0.0f) {
        wheel_hold_state=1;wheel_feedback_valid=0;return 0;
    }
    if (!sample->valid || !isfinite(sample->speed_cps) || !isfinite(sample->dt_ms) ||
        sample->dt_ms<=0.0f || sample->dt_ms>WHEEL_SPEED_ENCODER_TIMEOUT_MS) return fail(7);
    if (!sample->position_valid) return fail(10);
    wheel_feedback_valid=1;
    if (!have_reference) {
        wheel_ref_count=sample->position_count;reference_epoch=sample->position_epoch;
        have_reference=1;captured_reference=1;position_elapsed_ms=0;wheel_target_cps=0;
    }
    /* Absolute observation positions are bounded; use a wide intermediate so
     * malformed boundary inputs cannot overflow before the safety check. */
    {
        int64_t error=(int64_t)wheel_ref_count-(int64_t)sample->position_count;
        if (error<INT32_MIN || error>INT32_MAX) return fail(10);
        wheel_pos_error=(int32_t)error;
    }
    if (wheel_pos_kp==0.0f || wheel_speed_limit==0.0f) {
        wheel_target_cps=0;position_elapsed_ms=0;pid_reset(&position_pid);
    }
    if (wheel_target_cps>wheel_speed_limit) wheel_target_cps=wheel_speed_limit;
    if (wheel_target_cps< -wheel_speed_limit) wheel_target_cps=-wheel_speed_limit;
    speed_pid.output_min=-wheel_tilt_limit;speed_pid.output_max=wheel_tilt_limit;
    speed_pid.integral_limit=wheel_i_limit;
    /* A live reduction of the I limit must also clamp the old integral before
     * conditional antiwindup can retain it, including while output is saturated. */
    if (speed_pid.integral>wheel_i_limit) speed_pid.integral=wheel_i_limit;
    if (speed_pid.integral< -wheel_i_limit) speed_pid.integral=-wheel_i_limit;
    speed_pid.kp=wheel_speed_kp;speed_pid.ki=freeze_integral?0.0f:wheel_speed_ki;speed_pid.kd=0.0f;
    if (!have_frame || sample->sequence!=last_sequence) {
        if (!captured_reference && wheel_pos_kp>0.0f && wheel_speed_limit>0.0f)
            position_elapsed_ms+=sample->dt_ms;
        if (wheel_pos_kp>0.0f && wheel_speed_limit>0.0f && position_elapsed_ms>=WHEEL_POSITION_PERIOD_MS) {
            float error=(float)wheel_pos_error;
            float band=(float)WHEEL_POSITION_DEADBAND_COUNT;
            error=error>band?error-band:error< -band?error+band:0.0f;
            position_pid.kp=wheel_pos_kp;position_pid.ki=position_pid.kd=0;
            position_pid.integral_limit=0;
            position_pid.output_min=-wheel_speed_limit;position_pid.output_max=wheel_speed_limit;
            wheel_target_cps=pid_position_update(&position_pid,error,0.0f,position_elapsed_ms*0.001f);
            position_elapsed_ms=0;
        }
        output=pid_position_update(&speed_pid,wheel_target_cps,sample->speed_cps,sample->dt_ms*0.001f);
        have_frame=1;last_sequence=sample->sequence;
    } else output=speed_pid.output;
    /* A reduced menu limit also constrains a held output before the next frame. */
    if (output>wheel_tilt_limit) output=wheel_tilt_limit;
    if (output< -wheel_tilt_limit) output=-wheel_tilt_limit;
    wheel_speed_integral=speed_pid.integral;
    wheel_speed_error=wheel_target_cps-sample->speed_cps;
    wheel_tilt_offset=WHEEL_SPEED_TILT_SIGN*output;
    wheel_hold_state=2;
    return 1;
}
