#include "pid.h"

uint8_t cnt_1ms;
uint8_t cnt2_1ms;
uint16_t cnt3_1ms;

int16 real_speed;

uint8_t shacheflag;
uint8_t turn_flag;
volatile uint8_t cam_flag;
uint8_t tele_flag;

/* Legacy output names: unsuffixed = flywheel Pitch/Y; _p = wheel Roll/X.
 * dynamic_zero_roll and turn_k*_l/r retain the flywheel role;
 * dynamic_zero_pitch and turn_k*_p_l/r retain the wheel role. */
float cascade_vel_out;
float cascade_vel_out_p;
float cascade_vel_out_y;

float cascade_angle_out;
float cascade_angle_out_p;
float cascade_angle_out_y;

float cascade_angle_vel_out;
float cascade_angle_vel_out_p;
float cascade_angle_vel_out_y;

int16 costrained_flywheel_speed_left;
int16 costrained_flywheel_speed_right;
int16 costrained_wheel_speed;

Mono_params mono_params;
Mono_beacon mono_beacon;

pid_param_t cascade_vel_pitch;// 级联 PID 速度环参数。
pid_param_t cascade_vel_roll;
pid_param_t cascade_vel_yaw;
pid_param_t cascade_ang_pitch;// 级联 PID 角度环参数。
pid_param_t cascade_ang_roll;
pid_param_t cascade_ang_yaw;
pid_param_t cascade_ang_vel_pitch;// 级联 PID 角速度环参数。
pid_param_t cascade_ang_vel_roll;
pid_param_t cascade_ang_vel_yaw;

// 函数简介：将浮点数限制在指定的上下限范围内。
// 参数说明：amt 待限制的数值，low 下限，high 上限。
// 返回值：限制后的浮点数。
float constrain_float(float amt, float low, float high)
{
    return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

// 函数简介：计算角速度环 PID 输出。
// 参数说明：pid PID 参数，target_angle_vel 目标角速度，now_angle_vel 当前角速度。
// 返回值：角速度环控制输出。
float cascade_angle_vel(pid_param_t * pid, float target_angle_vel, float now_angle_vel)
{
    static float err;
    err = target_angle_vel - now_angle_vel;// 计算实际角速度与目标角速度的误差。

    // 累计误差用于积分项。
    pid->integrator += err;

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * constrain_float(pid->integrator, -3, 3);
    pid->out_d = pid->kd * (err-pid->last_error);

    pid->last_error = err;

    pid->out =( pid->out_p  + pid->out_i+ pid->out_d ) ;

    return pid->out;
}

// 计算级联角度环。

// 函数简介：计算角度环 PID 输出。
// 参数说明：pid PID 参数，target_angle 目标角度，now_angle 当前角度，gyro 当前陀螺仪角速度。
// 返回值：角度环控制输出。
float cascade_angle(pid_param_t * pid, float target_angle, float now_angle, float gyro)
{
    static float err;
    err = target_angle - now_angle;// 计算角度环控制量。

    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -30, 30);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * gyro;

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;

    return pid->out;

}

// 函数简介：根据角度误差计算 Yaw 角度环 PID 输出。
// 参数说明：pid PID 参数，err 当前角度误差。
// 返回值：Yaw 角度环控制输出。
float cascade_angle_y(pid_param_t * pid, float err)
{

    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -30, 30);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (err - pid->last_error);

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}
// 函数简介：计算增量式 Yaw 角度环 PID 输出。
// 参数说明：pid PID 参数，err 当前角度误差。
// 返回值：限制后的 Yaw 角度环控制输出。
float increment_cascade_angle_y(pid_param_t * pid, float err)
{

    pid->derivative = err-pid->last_error;

    pid->out_p = pid->kp * (err-pid->last_error);
    pid->out_i = pid->ki * err;
    pid->out_d = pid->kd * (pid->derivative-pid->last_derivative);

    pid->last_error = err;
    pid->last_derivative = pid->derivative;

    pid->out += pid->out_p  + pid->out_i+ pid->out_d  ;
    pid->out = constrain_float(pid->out,-180.0f,180.0f);

    return pid->out;
}

// 函数简介：计算速度环 PID 输出。
// 参数说明：pid PID 参数，target_vel 目标速度，now_vel 当前速度。
// 返回值：速度环控制输出。
float cascade_velocity(pid_param_t * pid, float target_vel, float now_vel)
{
    static float err;
    err = target_vel - now_vel;// 计算角速度环控制量。

    // 累计误差用于积分项。
    pid->integrator += err;

    pid->integrator = constrain_float(pid->integrator, -70, 70);

    pid->out_p = pid->kp * err;
    pid->out_i = pid->ki * pid->integrator;
    pid->out_d = pid->kd * (err - pid->last_error);

    pid->last_error = err;

    pid->out = pid->out_p + pid->out_i + pid->out_d;
    return pid->out;
}

// 函数简介：计算增量式角速度环 PID 输出。
// 参数说明：pid PID 参数，target_angle_vel 目标角速度，now_angle_vel 当前角速度。
// 返回值：限制后的角速度环控制输出。
float increment_cascade_angle_vel(pid_param_t * pid,float target_angle_vel, float now_angle_vel)
{
    static float err;

    err = target_angle_vel - now_angle_vel;
    pid->derivative = err-pid->last_error;

    pid->out_p = pid->kp * (err-pid->last_error);
    pid->out_i = pid->ki * err;
    pid->out_d = pid->kd * (pid->derivative-pid->last_derivative);

    pid->last_error = err;
    pid->last_derivative = pid->derivative;

    pid->out += pid->out_p  + pid->out_i+ pid->out_d  ;
    pid->out = constrain_float(pid->out,-7500.0f,7500.0f);

    return pid->out;
}

// 函数简介：初始化单轮车运动控制参数。
// 参数说明：无。
// 返回值：无。
void init_mono_params(void)
{
    mono_params.setspeed=0;
    mono_params.balance_angle_x=3.2f;
    mono_params.balance_angle_y=-0.3f;
    mono_params.dead_zone=0;
    mono_params.dead_zone_p=0;
    mono_params.turn_k1_l=8;
    mono_params.turn_k1_r=9;
    mono_params.turn_k1_p_l=-8;
    mono_params.turn_k1_p_r=-9;
    mono_params.fast_speed=0.17;
    mono_params.slow_speed=0.15;
}

// 函数简介：初始化摄像头信标和轨迹跟踪参数。
// 参数说明：无。
// 返回值：无。
void init_mono_beacon_params(void)
{
    mono_beacon.yaw_increment=1;
    mono_beacon.dynamic_zero_threshold=10;
    mono_beacon.track_threshold=50;
    mono_beacon.turn_threshold=110;
    mono_beacon.dynamic_mid_line_kp=0.8;
    mono_beacon.dynamic_mid_line_offset=41;
    mono_beacon.img_mid_line=93;
    mono_beacon.angle_err_max=60;
}

// 函数简介：设置各级联 PID 控制器的比例、积分和微分参数。
// 参数说明：无。
// 返回值：无。
void param_pid_set(void)
{
    /* Pitch 角速度环比例系数。 */
    cascade_ang_vel_pitch.kp = 5;
    /* Pitch 角速度环积分系数。 */
    cascade_ang_vel_pitch.ki = 1;
    /* Pitch 角速度环微分系数。 */
    cascade_ang_vel_pitch.kd = 0;

    /* Pitch 角度环比例系数。 */
    cascade_ang_pitch.kp = 6;
    /* Pitch 角度环积分系数。 */
    cascade_ang_pitch.ki = 0;
    /* Pitch 角度环微分系数。 */
    cascade_ang_pitch.kd = 0;

    /* Pitch 速度环比例系数。 */
    cascade_vel_pitch.kp = 0.5;
    /* Pitch 速度环积分系数。 */
    cascade_vel_pitch.ki = 0;
    /* Pitch 速度环微分系数。 */
    cascade_vel_pitch.kd = 0;

    /* Roll 角速度环比例系数。 */
    cascade_ang_vel_roll.kp = 25;
    /* Roll 角速度环积分系数。 */
    cascade_ang_vel_roll.ki = 0.5;
    /* Roll 角速度环微分系数。 */
    cascade_ang_vel_roll.kd = 0;

    /* Roll 角度环比例系数。 */
    cascade_ang_roll.kp = 6;
    /* Roll 角度环积分系数。 */
    cascade_ang_roll.ki = 0;
    /* Roll 角度环微分系数。 */
    cascade_ang_roll.kd = 0;

    /* Roll 速度环比例系数。 */
    cascade_vel_roll.kp = 50;
    /* Roll 速度环积分系数。 */
    cascade_vel_roll.ki = 0;
    /* Roll 速度环微分系数。 */
    cascade_vel_roll.kd = 0;

    /* Yaw 角速度环比例系数。 */
    cascade_ang_vel_yaw.kp= 80;
    /* Yaw 角速度环积分系数。 */
    cascade_ang_vel_yaw.ki= 0;
    /* Yaw 角速度环微分系数。 */
    cascade_ang_vel_yaw.kd= 0;

    /* Yaw 角度环比例系数。 */
    cascade_ang_yaw.kp= 2;
    /* Yaw 角度环积分系数。 */
    cascade_ang_yaw.ki= 0;
    /* Yaw 角度环微分系数。 */
    cascade_ang_yaw.kd= 0;
}

// 函数简介：初始化转弯识别及轨迹跟踪状态。
// 参数说明：无。
// 返回值：无。
void turn_info_init(void)
{
    turn_info.cx=0;
    turn_info.cy=0;
    turn_info.last_turn_flag=0;
    turn_info.turn_flag=0;
    turn_info.track_flag=0;
}

// 函数简介：使 Yaw 误差按固定步长平滑逼近目标角度误差。
// 参数说明：Yaw_err 当前 Yaw 误差，angle_err 目标角度误差。
// 返回值：更新后的 Yaw 误差。
float Yaw_err_update(float Yaw_err,float angle_err)
{
    float output;
    output=Yaw_err;
    if(output<angle_err)
    {
        output+=mono_beacon.yaw_increment;
        if(output>angle_err)
        {
            output=angle_err;
        }
    }
    else
    {
        output-=mono_beacon.yaw_increment;
        if(output<angle_err)
        {
            output=angle_err;
        }
    }
    return output;
}

//void track_turn_update(ImageInfo* imageinfo,Turn_info* turn_info)
//{
//    if(turn_info->turn_flag==left)
//    {
//        turn_info->track_flag=0;
//        mono_beacon.angle_err=-mono_beacon.angle_err_max;
//    }
//    if(turn_info->turn_flag==right)
//    {
//        turn_info->track_flag=0;
//        mono_beacon.angle_err=mono_beacon.angle_err_max;
//    }
//    if(imageinfo->numContours)
//    {
//        turn_info->last_turn_flag=turn_info->turn_flag;
//        turn_info->turn_flag=0;
//        if(turn_info->cy<mono_beacon.track_threshold)
//        {
//            mono_beacon.dynamic_mid_line=mono_beacon.img_mid_line;
//            mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//            if(turn_info->track_flag==0)
//            {
//                if(turn_info->last_turn_flag==right)
//                {
//                    turn_info->track_flag=right_left;
//                }
//                if(turn_info->last_turn_flag==left)
//                {
//                    turn_info->track_flag=left_right;
//                }
//                if(turn_info->last_turn_flag==0)
//                {
//                    turn_info->track_flag=right_left;
//                }
//            }
//        }
//        else
//        {
//            if(turn_info->track_flag==0)
//            {
//                if(turn_info->last_turn_flag==left)
//                {
//                    mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                    if(turn_info->cy>=mono_beacon.turn_threshold)
//                    {
//                        turn_info->turn_flag=right;
//                    }
//                }
//                if(turn_info->last_turn_flag==right||turn_info->last_turn_flag==0)
//                {
//                    mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                    if(turn_info->cy>=mono_beacon.turn_threshold)
//                    {
//                        turn_info->turn_flag=left;
//                    }
//                }
//            }
//            if(turn_info->track_flag==left_left)
//            {
//                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=left;
//                }
//            }
//            if(turn_info->track_flag==right_right)
//            {
//                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=right;
//                }
//            }
//            if(turn_info->track_flag==left_right)
//            {
//                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=right;
//                }
//            }
//            if(turn_info->track_flag==right_left)
//            {
//                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
//                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
//                if(turn_info->cy>=mono_beacon.turn_threshold)
//                {
//                    turn_info->turn_flag=left;
//                }
//            }
//        }
//    }
//}

// 函数简介：根据摄像头轮廓信息更新循迹误差和转弯状态。
// 参数说明：imageinfo 图像处理结果，turn_info 转弯及循迹状态信息。
// 返回值：无。
void track_turn_update(ImageInfo* imageinfo,Turn_info* turn_info)
{
    if(turn_info->turn_flag==left)
    {
        turn_info->last_turn_flag=turn_info->turn_flag;
        turn_info->track_flag=0;
        mono_beacon.angle_err=-mono_beacon.angle_err_max;
    }
    if(turn_info->turn_flag==right)
    {
        turn_info->last_turn_flag=turn_info->turn_flag;
        turn_info->track_flag=0;
        mono_beacon.angle_err=mono_beacon.angle_err_max;
    }
    if(imageinfo->numContours)
    {
        turn_info->turn_flag=0;
        if(turn_info->cy<mono_beacon.track_threshold)
        {
            mono_beacon.dynamic_mid_line=mono_beacon.img_mid_line;
            mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
            if(turn_info->track_flag==0)
            {
                if(turn_info->last_turn_flag==right)
                {
                    turn_info->track_flag=right_right;
                }
                if(turn_info->last_turn_flag==left||turn_info->last_turn_flag==0)
                {
                    turn_info->track_flag=left_left;
                }
            }
        }
        else
        {
            if(turn_info->track_flag==0)
            {
                if(turn_info->last_turn_flag==left||turn_info->last_turn_flag==0)
                {
                    mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                    if(turn_info->cy>=mono_beacon.turn_threshold)
                    {
                        turn_info->turn_flag=left;
                    }
                }
                if(turn_info->last_turn_flag==right)
                {
                    mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
                    mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                    if(turn_info->cy>=mono_beacon.turn_threshold)
                    {
                        turn_info->turn_flag=right;
                    }
                }
            }
            if(turn_info->track_flag==left_left)
            {
                mono_beacon.dynamic_mid_line=mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line-mono_beacon.dynamic_mid_line_offset;
                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                if(turn_info->cy>=mono_beacon.turn_threshold)
                {
                    turn_info->turn_flag=left;
                }
            }
            if(turn_info->track_flag==right_right)
            {
                mono_beacon.dynamic_mid_line=-mono_beacon.dynamic_mid_line_kp*turn_info->cy+mono_beacon.img_mid_line+mono_beacon.dynamic_mid_line_offset;
                mono_beacon.angle_err=turn_info->cx-mono_beacon.dynamic_mid_line;
                if(turn_info->cy>=mono_beacon.turn_threshold)
                {
                    turn_info->turn_flag=right;
                }
            }
        }
    }
}

// 函数简介：根据当前速度模式设置车辆目标速度。
// 参数说明：imageinfo 图像处理结果，turn_info 转弯及循迹状态信息。
// 返回值：无。
void speed_ctrl(ImageInfo* imageinfo,Turn_info* turn_info)
{
    if(mono_params.speed_flag==fast)
    {
        mono_params.setspeed=mono_params.fast_speed;
    }

    if(mono_params.speed_flag==slow)
    {
        mono_params.setspeed=mono_params.slow_speed;
    }
}

/* States: 0 off, 1 calibration, 2 running;
 * -1 IMU failure/stale, -2 tilt, -3 invalid parameter. Faults latch until Start. */
volatile uint8 roll_rate_test_enabled;
volatile uint8 ground_rate_test_enabled;
volatile uint8 dual_balance_enabled;
volatile uint8 rate_only_test;
volatile int32 control_fault_reason,control_fault_imu_status;
volatile uint32 control_fault_seq;
volatile float control_fault_dt_ms,control_startup_scale;
uint32 control_startup_ramp_ms=1000;
static uint32 control_startup_elapsed_ms;

/* Index 0: displayed Roll/flywheel; index 1: displayed Pitch/ground wheel. */
volatile float dual_angle[2],dual_target[2],dual_angle_error[2];
volatile float dual_rate[2],dual_target_rate[2],dual_rate_error[2],dual_pwm[2];

float ground_rate_test_kp=100.0f, ground_rate_test_limit=4000.0f, ground_rate_test_sign=1.0f;
float ground_angle_kp=10.0f, ground_angle_rate_limit=30.0f;
volatile uint32 roll_rate_test_sequence;
volatile int32 roll_rate_test_state;
volatile float roll_rate_test_actual, roll_rate_test_error, roll_rate_test_pwm;
volatile float pitch_balance_target_rate, pitch_balance_error;
volatile float pitch_balance_actual_angle, pitch_balance_target_angle;
float pitch_balance_kp = 50.0f;
float pitch_balance_rate_limit = 80.0f;
float roll_rate_test_kp = 35.0f;
float roll_rate_test_limit = 4000.0f;
float roll_rate_test_sign = 1.0f;
static uint8 rate_divider;
static uint32 rate_stale_ms, rate_sequence;

// 函数简介：停止飞轮角速度测试并清除测试状态和输出。
// 参数说明：无。
// 返回值：无。
void roll_rate_test_stop(void)
{
    roll_rate_test_enabled = 0;
    ground_rate_test_enabled = 0;
    dual_balance_enabled=0;rate_only_test=0;
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
    if(control_startup_elapsed_ms<5000u)control_startup_elapsed_ms+=5u;
    control_startup_scale=control_startup_ramp_ms==0?1.0f:
        constrain_float((float)control_startup_elapsed_ms/(float)control_startup_ramp_ms,0.0f,1.0f);
}

void dual_balance_arm(void)
{
    ground_rate_test_arm();
    dual_balance_enabled=1;
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
    a[0].angle=imu_pitch_deg; a[0].rate=attitude_state.gyro.y;
    a[0].target=mono_params.balance_angle_y;
    a[0].angle_kp=pitch_balance_kp; a[0].rate_limit=pitch_balance_rate_limit;
    a[0].rate_kp=roll_rate_test_kp; a[0].pwm_limit=roll_rate_test_limit; a[0].sign=roll_rate_test_sign;
    a[1].angle=imu_roll_deg; a[1].rate=attitude_state.gyro.x;
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
        } else if (!seq || !isfinite(a[0].angle) || !isfinite(a[1].angle) ||
                   !isfinite(a[0].rate) || !isfinite(a[1].rate)) control_latch_fault(-1,seq?3:2);
        else if (fabsf(a[0].angle)>30 || fabsf(a[1].angle)>30) control_latch_fault(-2,5);
        else if (!dual_axis_parameters_valid(&a[0]) || !dual_axis_parameters_valid(&a[1]) || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (seq==rate_sequence) rate_stale_ms+=5;
            else {rate_stale_ms=0;rate_sequence=seq;}
            if (rate_stale_ms>=25) control_latch_fault(-1,4);
            else {
                roll_rate_test_state=2;
                control_startup_advance();
                for(unsigned i=0;i<2;i++) {
                    dual_target_rate[i]=constrain_float(a[i].angle_kp*dual_angle_error[i],-a[i].rate_limit,a[i].rate_limit);
                    dual_rate_error[i]=dual_target_rate[i]-a[i].rate;
                    output[i]=(int)roundf(constrain_float(a[i].sign*a[i].rate_kp*dual_rate_error[i],-a[i].pwm_limit*control_startup_scale,a[i].pwm_limit*control_startup_scale));
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
    small_driver_set_duty((int16)output[0],(int16)(-output[0]));
    motor_m_ctrl(output[1]);
}

static void roll_rate_test_update(void)
{
    float rate, roll, pitch, kp, limit, direction;
    float angle_kp, rate_limit, balance_angle;
    uint32 sequence, irq;
    int32 status;
    int output = 0;
    if (++rate_divider < 5) return;
    rate_divider = 0;
    ++roll_rate_test_sequence;
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
        else if (fabsf(ground_rate_test_enabled ? roll : pitch) > 30.0f)
            control_latch_fault(-2,5);
        else if (!isfinite(kp) || kp < 0.0f || kp > 500.0f ||
                 !isfinite(limit) || limit < 0.0f || limit > 4000.0f ||
                 (direction != 1.0f && direction != -1.0f) ||
                 !isfinite(angle_kp) || angle_kp < 0.0f || angle_kp > 100.0f ||
                 !isfinite(rate_limit) || rate_limit <= 0.0f || rate_limit > 100.0f ||
                 !isfinite(balance_angle) || fabsf(balance_angle) > 20.0f || control_startup_ramp_ms>5000u)
            control_latch_fault(-3,6);
        else {
            if (sequence == rate_sequence) rate_stale_ms += 5;
            else { rate_stale_ms = 0; rate_sequence = sequence; }
            if (rate_stale_ms >= 25) control_latch_fault(-1,4);
            else {
                roll_rate_test_state = 2;
                control_startup_advance();
                pitch_balance_error = balance_angle - (ground_rate_test_enabled ? roll : pitch);
                pitch_balance_target_rate = constrain_float(angle_kp * pitch_balance_error, -rate_limit, rate_limit);
                roll_rate_test_error = pitch_balance_target_rate - rate;
                output = (int)roundf(constrain_float(direction * kp * roll_rate_test_error, -limit*control_startup_scale, limit*control_startup_scale));
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

// 函数简介：执行单轮车级联控制，包括速度、姿态、转向和电机输出控制。
// 参数说明：无。
// 返回值：无。
void cascade_ctrl(void)
{
    if (roll_rate_test_enabled) { roll_rate_test_update(); return; }
    if (imu_attitude_status!=IMU_ATTITUDE_RUNNING) {
        clear_all_cascade_out();
        small_driver_set_duty(0,0);
        motor_m_ctrl(0);
        return;
    }
    cnt_1ms++;
    cnt2_1ms++;
    if(cnt_1ms==20)
    {
        cnt_1ms=0;
        real_mid_speed = encoder_get_count(TIM6_ENCODER);
        encoder_clear_count(TIM6_ENCODER);
        real_speed = (motor_value.receive_left_speed_data - motor_value.receive_right_speed_data) * 0.005f;

        if(cam_flag)
        {
            speed_ctrl(&imageinfo,&turn_info);
        }
        if(tele_flag)
        {
            if(mono_params.speed_flag==fast)
            {
                mono_params.setspeed=mono_params.fast_speed*0.00390625f*L_UpDown;
            }

            if(mono_params.speed_flag==slow)
            {
                mono_params.setspeed=mono_params.slow_speed*0.00390625f*L_UpDown;
            }
        }

        cascade_vel_out = cascade_velocity(&cascade_vel_pitch, 0.0f, real_speed);//fly_wheel
        cascade_vel_out_p = cascade_velocity(&cascade_vel_roll, mono_params.setspeed, real_mid_speed*0.00069f);
    }

    if(cnt2_1ms==5)
    {
        cnt2_1ms=0;
        if(cam_flag){track_turn_update(&imageinfo,&turn_info);}

        if(tele_flag){mono_beacon.angle_err=(float)0.01*mono_beacon.angle_err_max*R_Around;}

        mono_beacon.angle_err=constrain_float(mono_beacon.angle_err,-mono_beacon.angle_err_max,mono_beacon.angle_err_max);
        mono_beacon.Yaw_err = Yaw_err_update(mono_beacon.Yaw_err,mono_beacon.angle_err);

        if(mono_params.speed_flag==fast)
        {
            if(fabs(mono_beacon.Yaw_err)>=mono_beacon.dynamic_zero_threshold)
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_params.turn_k1_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_params.turn_k1_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }
            else
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k1_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k1_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }

            if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_roll=mono_params.turn_k1_l*mono_beacon.Yaw_err*DEG2RAD;}
            if(mono_beacon.Yaw_err>0){mono_beacon.dynamic_zero_roll=mono_params.turn_k1_r*mono_beacon.Yaw_err*DEG2RAD;}
        }

        if(mono_params.speed_flag==slow)
        {
            if(fabs(mono_beacon.Yaw_err)>=mono_beacon.dynamic_zero_threshold)
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_params.turn_k2_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_params.turn_k2_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }
            else
            {
                if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k2_p_l*fabs(mono_beacon.Yaw_err)*DEG2RAD;}
                else{mono_beacon.dynamic_zero_pitch=mono_beacon.dynamic_zero_threshold*0.0166667f*mono_params.turn_k2_p_r*mono_beacon.Yaw_err*DEG2RAD;}
            }

            if(mono_beacon.Yaw_err<0){mono_beacon.dynamic_zero_roll=mono_params.turn_k2_l*mono_beacon.Yaw_err*DEG2RAD;}
            if(mono_beacon.Yaw_err>0){mono_beacon.dynamic_zero_roll=mono_params.turn_k2_r*mono_beacon.Yaw_err*DEG2RAD;}

        }

        cascade_angle_out = cascade_angle(&cascade_ang_pitch, -cascade_vel_out + mono_params.balance_angle_y-mono_beacon.dynamic_zero_roll, attitude_state.pitch, -attitude_state.gyro.y);// 计算级联 PID 角度环输出。
        cascade_angle_out_p = cascade_angle(&cascade_ang_roll, -cascade_vel_out_p + mono_params.balance_angle_x-mono_beacon.dynamic_zero_pitch, attitude_state.roll, -attitude_state.gyro.x);
        cascade_angle_out_y = cascade_angle_y(&cascade_ang_yaw,mono_beacon.Yaw_err);
    }

    /* IMU samples are published by the 5ms pipeline; no nested SPI reads. */

    cascade_angle_vel_out   = increment_cascade_angle_vel(&cascade_ang_vel_pitch, cascade_angle_out, attitude_state.gyro.y); // 计算级联 PID 角速度环输出。
    cascade_angle_vel_out_p = increment_cascade_angle_vel(&cascade_ang_vel_roll, cascade_angle_out_p, attitude_state.gyro.x);// 累计误差。
    cascade_angle_vel_out_y = cascade_angle_vel(&cascade_ang_vel_yaw, cascade_angle_out_y, -attitude_state.gyro.z);

//    cascade_angle_vel_out +=(cascade_angle_vel_out>0?mono_params.dead_zone:-mono_params.dead_zone);
//    cascade_angle_vel_out_p +=(cascade_angle_vel_out_p>0?mono_params.dead_zone_p:-mono_params.dead_zone_p);

    if(fabsf(attitude_state.pitch)>25.0f || fabsf(attitude_state.roll)>30.0f)
    {
        cnt3_1ms++;
    }
    else
    {
        cnt3_1ms=0;
    }

    if(cnt3_1ms==1000)
    {
        cnt3_1ms=0;
        shacheflag=0;
    }

    costrained_flywheel_speed_left = (int)round(constrain_float(cascade_angle_vel_out-cascade_angle_vel_out_y*turn_flag, -7500.0f, 7500.0f)*shacheflag);
    costrained_flywheel_speed_right = (int)round(constrain_float(cascade_angle_vel_out+cascade_angle_vel_out_y*turn_flag, -7500.0f, 7500.0f)*shacheflag);
    costrained_wheel_speed = (int)round(constrain_float(cascade_angle_vel_out_p, -7500.0f, 7500.0f)*shacheflag);

    small_driver_set_duty(costrained_flywheel_speed_left , -costrained_flywheel_speed_right);
    motor_m_ctrl(costrained_wheel_speed);
}

// 函数简介：清零级联控制器内部输出及电机控制输出。
// 参数说明：无。
// 返回值：无。
void clear_all_cascade_out(void)
{
    cascade_ang_vel_pitch.out=0;
    cascade_ang_vel_roll.out=0;
    cascade_ang_vel_yaw.out=0;

    cascade_ang_pitch.out=0;
    cascade_ang_roll.out=0;
    cascade_ang_yaw.out=0;

    cascade_vel_pitch.out=0;
    cascade_vel_roll.out=0;

    cascade_vel_out=0;
    cascade_vel_out_p=0;
    cascade_vel_out_y=0;

    cascade_angle_out=0;
    cascade_angle_out_p=0;
    cascade_angle_out_y=0;

    cascade_angle_vel_out=0;
    cascade_angle_vel_out_p=0;
    cascade_angle_vel_out_y=0;

    costrained_flywheel_speed_left=0;
    costrained_flywheel_speed_right=0;
    costrained_wheel_speed=0;
}
