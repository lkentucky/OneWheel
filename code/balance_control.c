/* Upright controller, extracted from upstream pid.c (GPL-3.0-or-later).
 * Modes 3..7 retain their numerical gains, signs and angle LPF; 8 is raw PWM.
 */
#include "balance_control.h"
#include "pid.h"

uint8_t shacheflag;
int16 costrained_flywheel_speed_left, costrained_flywheel_speed_right, costrained_wheel_speed;
float cascade_angle_out,cascade_angle_out_p,cascade_angle_vel_out,cascade_angle_vel_out_p;
Mono_params mono_params;
float constrain_float(float v,float lo,float hi) { return v<lo?lo:v>hi?hi:v; }
/* One private PID instance per axis and loop; no shared error history. */
static pid_controller_t angle_pid[2],rate_pid[2];
static float controller_dt_s(void) {return (float)APP_IMU_PERIOD_US*0.000001f;}
static void balance_pid_reset(void)
{
    unsigned i;for(i=0;i<2;++i){pid_reset(&angle_pid[i]);pid_reset(&rate_pid[i]);}
}
static float balance_angle_pid(unsigned axis,float target,float angle,float kp,float rate_limit)
{
    pid_controller_t *p=&angle_pid[axis];
    p->kp=kp;p->ki=axis?GROUND_ANGLE_KI:FLY_ANGLE_KI;p->kd=axis?GROUND_ANGLE_KD:FLY_ANGLE_KD;
    p->integral_limit=BALANCE_ANGLE_I_LIMIT_DPS;
    p->output_min=-rate_limit;p->output_max=rate_limit;
#if BALANCE_ANGLE_PID_INCREMENTAL
    return pid_incremental_update(p,target,angle,controller_dt_s());
#else
    return pid_position_update(p,target,angle,controller_dt_s());
#endif
}
static float balance_rate_pid(unsigned axis,float target,float rate,float kp,float pwm_limit,float sign)
{
    pid_controller_t *p=&rate_pid[axis];
    p->kp=kp;p->ki=axis?GROUND_RATE_KI:FLY_RATE_KI;p->kd=axis?GROUND_RATE_KD:FLY_RATE_KD;
    p->integral_limit=BALANCE_RATE_I_LIMIT_PWM;
    p->output_min=-pwm_limit;p->output_max=pwm_limit;
#if BALANCE_RATE_PID_INCREMENTAL
    return sign*pid_incremental_update(p,target,rate,controller_dt_s());
#else
    return sign*pid_position_update(p,target,rate,controller_dt_s());
#endif
}

/* States: 0 off, 1 calibration, 2 running;
 * -1 IMU failure/stale, -2 tilt, -3 invalid parameter. Faults latch until Start. */
volatile uint8 roll_rate_test_enabled;
volatile uint8 ground_rate_test_enabled;
volatile uint8 dual_balance_enabled;
volatile uint8 rate_only_test;
volatile uint8 pwm_debug_enabled;
volatile float pwm_debug_fly_left, pwm_debug_fly_right, pwm_debug_ground;
volatile int32 control_fault_reason,control_fault_imu_status;
volatile uint32 control_fault_seq;
volatile float control_fault_dt_ms,control_startup_scale;
uint32 control_startup_ramp_ms=BALANCE_STARTUP_RAMP_MS;
static uint32 control_startup_elapsed_ms;

/* Index 0: displayed Roll/flywheel; index 1: displayed Pitch/ground wheel. */
volatile float dual_angle[2],dual_target[2],dual_angle_error[2];
volatile float dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];

/* Compiled defaults live in config.h; valid upright Flash values may override. */
float ground_rate_test_kp=GROUND_RATE_KP, ground_rate_test_limit=GROUND_PWM_LIMIT, ground_rate_test_sign=GROUND_OUTPUT_SIGN;
float ground_angle_kp=GROUND_ANGLE_KP, ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
volatile uint32 roll_rate_test_sequence;
volatile int32 roll_rate_test_state;
volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
volatile float pitch_balance_target_rate, pitch_balance_error;
volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
float pitch_balance_kp = FLY_ANGLE_KP;
float pitch_balance_rate_limit = FLY_RATE_LIMIT_DPS;
float roll_rate_test_kp = FLY_RATE_KP;
float roll_rate_test_limit = FLY_PWM_LIMIT;
float roll_rate_test_sign = FLY_OUTPUT_SIGN;
static uint8 rate_divider;
static uint32 rate_stale_ms, rate_sequence;
/* Preserve upstream's angle LPF. Default tau=60ms, about 2.65Hz corner;
 * this adds phase lag and requires physical tuning, not proof of stability. */
#define ANGLE_LPF_TAU_S BALANCE_ANGLE_LPF_TAU_S
static float angle_lpf[2];
static float angle_lpf_update(float *state, float value)
{
    const float dt = (float)APP_IMU_PERIOD_US*0.000001f;
    if(ANGLE_LPF_TAU_S<=0.0f) {*state=value;return value;}
    *state += (value - *state) * (dt / (ANGLE_LPF_TAU_S + dt));
    return *state;
}

// 函数简介：停止飞轮角速度测试并清除测试状态和输出。
// 参数说明：无。
// 返回值：无。
void roll_rate_test_stop(void)
{
    balance_pid_reset();
    roll_rate_test_enabled = 0;
    ground_rate_test_enabled = 0;
    dual_balance_enabled=0;rate_only_test=0;
    pwm_debug_enabled=0;
    pwm_debug_fly_left=pwm_debug_fly_right=pwm_debug_ground=0.0f;
    control_fault_reason=control_fault_imu_status=0;control_fault_seq=0;
    control_fault_dt_ms=control_startup_scale=0;control_startup_elapsed_ms=0;
    for (unsigned i=0;i<2;i++) {
        dual_angle[i]=dual_target[i]=dual_angle_error[i]=0;
        dual_rate[i]=dual_target_rate[i]=dual_rate_error[i]=dual_pwm[i]=0;
    }
    roll_rate_test_state = 0;
    roll_rate_test_actual = roll_rate_test_error = roll_rate_test_pwm = 0.0f;
    pitch_balance_target_rate = pitch_balance_error = 0.0f;
    pitch_balance_actual_angle = pitch_balance_target_angle = 0.0f;
    rate_divider = 0;
    rate_stale_ms = 0;
}

// 函数简介：启动飞轮角速度测试。
// 参数说明：无。
// 返回值：无。
void roll_rate_test_arm(void)
{
    roll_rate_test_stop();
    rate_sequence = imu_sample_sequence;
    /* 角度低通以当前姿态为初值，避免起跑瞬间的大误差。 */
    angle_lpf[0] = imu_pitch_deg; angle_lpf[1] = imu_roll_deg;
    roll_rate_test_state = 1;
    roll_rate_test_enabled = 1;
}

// 函数简介：周期性更新飞轮角速度测试控制量并驱动电机。
// 参数说明：无。
// 返回值：无。
void ground_rate_test_arm(void)
{
    roll_rate_test_arm();
    ground_rate_test_enabled=1;
}

void roll_rate_only_arm(void) {roll_rate_test_arm();rate_only_test=1;}
void ground_rate_only_arm(void) {ground_rate_test_arm();rate_only_test=1;}
static void control_latch_fault(int32 state,int32 reason)
{
    uint32 irq=interrupt_global_disable();
    roll_rate_test_state=state;control_fault_reason=reason;
    control_fault_imu_status=imu_attitude_status;control_fault_seq=imu_sample_sequence;
    control_fault_dt_ms=imu_sample_dt_ms;
    interrupt_global_enable(irq);
}
static void control_startup_advance(void)
{
    if(control_startup_elapsed_ms<5000u)control_startup_elapsed_ms+=APP_IMU_PERIOD_US/1000u;
    control_startup_scale=control_startup_ramp_ms==0?1.0f:
        constrain_float((float)control_startup_elapsed_ms/(float)control_startup_ramp_ms,0.0f,1.0f);
}

void dual_balance_arm(void)
{
    ground_rate_test_arm();
    dual_balance_enabled=1;
}

/* Bench mode: direct signed PWM, no attitude controller or IMU interlock.
 * Arm/Stop zero all three commands. Only the control ISR owns active outputs. */
void pwm_debug_arm(void)
{
    roll_rate_test_stop();
    pwm_debug_enabled=1;
    roll_rate_test_enabled=1;
    roll_rate_test_state=2;
}

static void pwm_debug_update(void)
{
    float left,right,ground;
    int output[3]={0,0,0};
    uint32 irq=interrupt_global_disable();
    left=pwm_debug_fly_left;right=pwm_debug_fly_right;ground=pwm_debug_ground;
    interrupt_global_enable(irq);
    if (roll_rate_test_state==2) {
        if (!shacheflag) roll_rate_test_state=0;
        else if (!isfinite(left) || !isfinite(right) || !isfinite(ground) ||
                 !isfinite(DEBUG_PWM_LIMIT) || DEBUG_PWM_LIMIT<0.0f || DEBUG_PWM_LIMIT>4000.0f ||
                 fabsf(left)>DEBUG_PWM_LIMIT || fabsf(right)>DEBUG_PWM_LIMIT ||
                 fabsf(ground)>DEBUG_PWM_LIMIT || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            control_startup_advance();
            float limit=DEBUG_PWM_LIMIT*control_startup_scale;
            output[0]=(int)roundf(constrain_float(left,-limit,limit));
            output[1]=(int)roundf(constrain_float(right,-limit,limit));
            output[2]=(int)roundf(constrain_float(ground,-limit,limit));
        }
    }
    for(unsigned i=0;i<2;i++) {
        dual_target[i]=dual_angle_error[i]=0.0f;
        dual_target_rate[i]=dual_rate_error[i]=0.0f;
    }
    /* Compatibility channels: Roll PWM is LEFT only in Mode8, Pitch is ground.
     * motor_left_pwm/motor_right_pwm expose the actual independent commands. */
    dual_pwm[0]=(float)output[0];dual_pwm[1]=(float)output[2];
    /* Optional measurements remain visible; they never gate raw PWM. */
    dual_angle[0]=imu_pitch_deg;dual_angle[1]=imu_roll_deg;
    dual_rate[0]=attitude_state.gyro.y;dual_rate[1]=attitude_state.gyro.x;
    roll_rate_test_pwm=(float)output[2];
    cascade_angle_out=cascade_angle_out_p=0.0f;
    cascade_angle_vel_out=(float)output[0];cascade_angle_vel_out_p=(float)output[2];
    costrained_flywheel_speed_left=(int16)output[0];
    /* Preserve the legacy internal right-sign convention used by telemetry. */
    costrained_flywheel_speed_right=(int16)-output[1];
    costrained_wheel_speed=(int16)output[2];
    small_driver_set_duty((int16)output[0],(int16)(-output[1]));
    motor_m_ctrl(output[2]);
}

typedef struct {
    float angle,rate,target,angle_kp,rate_limit,rate_kp,pwm_limit,sign;
} balance_axis_snapshot;
static int dual_axis_parameters_valid(const balance_axis_snapshot *a)
{
    return isfinite(a->target) && fabsf(a->target)<=20 &&
        isfinite(a->angle_kp) && a->angle_kp>=0 && a->angle_kp<=100 &&
        isfinite(a->rate_kp) && a->rate_kp>=0 && a->rate_kp<=500 &&
        isfinite(a->rate_limit) && a->rate_limit>0 && a->rate_limit<=100 &&
        isfinite(a->pwm_limit) && a->pwm_limit>=0 && a->pwm_limit<=4000 &&
        (a->sign==1 || a->sign==-1);
}
static void dual_balance_update(void)
{
    balance_axis_snapshot a[2];
    uint32 seq,irq; int32 status;
    int output[2]={0,0};
    irq=interrupt_global_disable();
    a[0].angle=angle_lpf_update(&angle_lpf[0],imu_pitch_deg); a[0].rate=attitude_state.gyro.y;
    a[0].target=mono_params.balance_angle_y;
    a[0].angle_kp=pitch_balance_kp; a[0].rate_limit=pitch_balance_rate_limit;
    a[0].rate_kp=roll_rate_test_kp; a[0].pwm_limit=roll_rate_test_limit; a[0].sign=roll_rate_test_sign;
    a[1].angle=angle_lpf_update(&angle_lpf[1],imu_roll_deg); a[1].rate=attitude_state.gyro.x;
    a[1].target=mono_params.balance_angle_x;
    a[1].angle_kp=ground_angle_kp; a[1].rate_limit=ground_angle_rate_limit;
    a[1].rate_kp=ground_rate_test_kp; a[1].pwm_limit=ground_rate_test_limit; a[1].sign=ground_rate_test_sign;
    seq=imu_sample_sequence; status=imu_attitude_status;
    interrupt_global_enable(irq);
    for(unsigned i=0;i<2;i++) {
        dual_angle[i]=a[i].angle;dual_rate[i]=a[i].rate;dual_target[i]=a[i].target;
        dual_angle_error[i]=a[i].target-a[i].angle;
        dual_target_rate[i]=dual_rate_error[i]=dual_pwm[i]=0;
    }
    if (roll_rate_test_state==1 || roll_rate_test_state==2) {
        if (!shacheflag) roll_rate_test_state=0;
        else if (status<0) control_latch_fault(-1,1);
        else if (status==IMU_ATTITUDE_CALIBRATING) {
            roll_rate_test_state=1;rate_stale_ms=0;rate_sequence=seq;
        } else if (!seq || !isfinite(imu_roll_deg) || !isfinite(imu_pitch_deg) || !isfinite(a[0].angle) || !isfinite(a[1].angle) ||
                   !isfinite(a[0].rate) || !isfinite(a[1].rate)) control_latch_fault(-1,seq?3:2);
        else if (fabsf(imu_pitch_deg)>BALANCE_TILT_LIMIT_DEG || fabsf(imu_roll_deg)>BALANCE_TILT_LIMIT_DEG) control_latch_fault(-2,5);
        else if (!dual_axis_parameters_valid(&a[0]) || !dual_axis_parameters_valid(&a[1]) || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (seq==rate_sequence) rate_stale_ms+=APP_IMU_PERIOD_US/1000u;
            else {rate_stale_ms=0;rate_sequence=seq;}
            if (rate_stale_ms>=BALANCE_IMU_STALE_MS) control_latch_fault(-1,4);
            else {
                roll_rate_test_state=2;
                control_startup_advance();
                for(unsigned i=0;i<2;i++) {
                    dual_target_rate[i]=balance_angle_pid(i,a[i].target,a[i].angle,a[i].angle_kp,a[i].rate_limit);
                    dual_rate_error[i]=dual_target_rate[i]-a[i].rate;
                    output[i]=(int)roundf(balance_rate_pid(i,dual_target_rate[i],a[i].rate,a[i].rate_kp,a[i].pwm_limit*control_startup_scale,a[i].sign));
                    dual_pwm[i]=(float)output[i];
                }
            }
        }
    }
    /* Generic test channels continue to describe the ground/Pitch axis. */
    pitch_balance_actual_angle=dual_angle[1];pitch_balance_target_angle=dual_target[1];
    pitch_balance_error=dual_angle_error[1];pitch_balance_target_rate=dual_target_rate[1];
    roll_rate_test_actual=dual_rate[1];roll_rate_test_error=dual_rate_error[1];roll_rate_test_pwm=dual_pwm[1];
    cascade_angle_out=dual_target_rate[0];cascade_angle_out_p=dual_target_rate[1];
    cascade_angle_vel_out=(float)output[0];cascade_angle_vel_out_p=(float)output[1];
    costrained_flywheel_speed_left=costrained_flywheel_speed_right=(int16)output[0];
    costrained_wheel_speed=(int16)output[1];
    small_driver_set_duty((int16)output[0],-(int16)(output[0]));
    motor_m_ctrl(output[1]);
}

static void roll_rate_test_update(void)
{
    float rate, roll, pitch, kp, limit, direction;
    float angle_kp, rate_limit, balance_angle;
    uint32 sequence, irq;
    int32 status;
    int output = 0;
    if (++rate_divider < APP_CONTROL_DIVIDER) return;
    rate_divider = 0;
    ++roll_rate_test_sequence;
    if (pwm_debug_enabled) { pwm_debug_update(); return; }
    if (dual_balance_enabled) { dual_balance_update(); return; }
    irq = interrupt_global_disable();
    rate = ground_rate_test_enabled ? attitude_state.gyro.x : attitude_state.gyro.y;
    roll = imu_roll_deg;
    pitch = imu_pitch_deg;
    sequence = imu_sample_sequence;
    status = imu_attitude_status;
    kp = ground_rate_test_enabled ? ground_rate_test_kp : roll_rate_test_kp;
    limit = ground_rate_test_enabled ? ground_rate_test_limit : roll_rate_test_limit;
    direction = ground_rate_test_enabled ? ground_rate_test_sign : roll_rate_test_sign;
    angle_kp = rate_only_test ? 0.0f : (ground_rate_test_enabled ? ground_angle_kp : pitch_balance_kp);
    rate_limit = ground_rate_test_enabled ? ground_angle_rate_limit : pitch_balance_rate_limit;
    balance_angle = rate_only_test ? 0.0f : (ground_rate_test_enabled ? mono_params.balance_angle_x : mono_params.balance_angle_y);
    interrupt_global_enable(irq);
    roll_rate_test_actual = rate;
    pitch_balance_actual_angle = ground_rate_test_enabled ? roll : pitch;
    pitch_balance_target_angle = balance_angle;
    pitch_balance_target_rate = pitch_balance_error = 0.0f;
    roll_rate_test_error = -rate;
    if (roll_rate_test_state == 1 || roll_rate_test_state == 2) {
        if (!shacheflag) roll_rate_test_state = 0;
        else if (status < 0) control_latch_fault(-1,1);
        else if (status == IMU_ATTITUDE_CALIBRATING) {
            /* Calibration never drives either flywheel. */
            roll_rate_test_state = 1;
            rate_stale_ms = 0;
            rate_sequence = sequence;
        } else if (!sequence || !isfinite(rate) || !isfinite(roll) || !isfinite(pitch))
            control_latch_fault(-1,sequence?3:2);
        /* Display labels are swapped: internal pitch is the tested Roll. */
        else if (fabsf(ground_rate_test_enabled ? roll : pitch) > BALANCE_TILT_LIMIT_DEG)
            control_latch_fault(-2,5);
        else if (!isfinite(kp) || kp < 0.0f || kp > 500.0f ||
                 !isfinite(limit) || limit < 0.0f || limit > 4000.0f ||
                 (direction != 1.0f && direction != -1.0f) ||
                 !isfinite(angle_kp) || angle_kp < 0.0f || angle_kp > 100.0f ||
                 !isfinite(rate_limit) || rate_limit <= 0.0f || rate_limit > 100.0f ||
                 !isfinite(balance_angle) || fabsf(balance_angle) > 20.0f || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (sequence == rate_sequence) rate_stale_ms += APP_IMU_PERIOD_US/1000u;
            else { rate_stale_ms = 0; rate_sequence = sequence; }
            if (rate_stale_ms >= BALANCE_IMU_STALE_MS) control_latch_fault(-1,4);
            else {
                roll_rate_test_state = 2;
                control_startup_advance();
                /* 索引与双轴路径一致：0=pitch(飞轮)，1=roll(地轮)。 */
                pitch_balance_error = balance_angle - angle_lpf_update(
                    &angle_lpf[ground_rate_test_enabled ? 1u : 0u],
                    ground_rate_test_enabled ? roll : pitch);
                pitch_balance_target_rate = rate_only_test ? 0.0f : balance_angle_pid(ground_rate_test_enabled?1u:0u,balance_angle,balance_angle-pitch_balance_error,angle_kp,rate_limit);
                roll_rate_test_error = pitch_balance_target_rate - rate;
                output = (int)roundf(balance_rate_pid(ground_rate_test_enabled?1u:0u,pitch_balance_target_rate,rate,kp,limit*control_startup_scale,direction));
            }
        }
    }
    roll_rate_test_pwm = (float)output;
    cascade_angle_out = ground_rate_test_enabled ? 0.0f : pitch_balance_target_rate;
    cascade_angle_vel_out = ground_rate_test_enabled ? 0.0f : (float)output;
    cascade_angle_vel_out_p = ground_rate_test_enabled ? (float)output : 0.0f;
    costrained_flywheel_speed_left = ground_rate_test_enabled ? 0 : (int16)output;
    costrained_flywheel_speed_right = ground_rate_test_enabled ? 0 : (int16)output;
    costrained_wheel_speed = ground_rate_test_enabled ? (int16)output : 0;
    small_driver_set_duty(costrained_flywheel_speed_left,-costrained_flywheel_speed_right);
    motor_m_ctrl(costrained_wheel_speed);
}


void balance_control_tick_1ms(void)
{
    if (roll_rate_test_enabled) roll_rate_test_update();
    else { small_driver_set_duty(0,0); motor_m_ctrl(0); }
}
void clear_all_cascade_out(void)
{
    cascade_angle_out=cascade_angle_out_p=0;
    cascade_angle_vel_out=cascade_angle_vel_out_p=0;
    costrained_flywheel_speed_left=costrained_flywheel_speed_right=costrained_wheel_speed=0;
}
void balance_control_defaults(void)
{
    roll_rate_test_stop(); clear_all_cascade_out();
    mono_params.balance_angle_x=BALANCE_ZERO_X_DEG;
    mono_params.balance_angle_y=BALANCE_ZERO_Y_DEG;
    roll_rate_test_kp=FLY_RATE_KP;roll_rate_test_limit=FLY_PWM_LIMIT;roll_rate_test_sign=FLY_OUTPUT_SIGN;
    pitch_balance_kp=FLY_ANGLE_KP;pitch_balance_rate_limit=FLY_RATE_LIMIT_DPS;
    ground_rate_test_kp=GROUND_RATE_KP;ground_rate_test_limit=GROUND_PWM_LIMIT;ground_rate_test_sign=GROUND_OUTPUT_SIGN;
    ground_angle_kp=GROUND_ANGLE_KP;ground_angle_rate_limit=GROUND_RATE_LIMIT_DPS;
    control_startup_ramp_ms=BALANCE_STARTUP_RAMP_MS;
}
