/* Ported from Kongmengwuwei/ASC_SummerTraining_Unicycle @ 0eed51b.
 * Changes: namespaced portable interface. See ASC_PORT.md and THIRD_PARTY.md. */
#include "asc_pid.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     浮点限幅
// 参数说明     amt/low/high    输入值与上下限
// 返回参数     float           限幅后的值
// 使用示例     offset = asc_clamp(offset, -limit, limit);
//-------------------------------------------------------------------------------------------------------------------
float asc_clamp(float amt, float low, float high)
{
    return (amt < low) ? low : ((amt > high) ? high : amt);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入增益并清掉历史状态
// 参数说明     p/kp/ki/kd/imax PID 实例、三个增益和积分限幅
// 返回参数     void
// 使用示例     asc_pid_set(&p_vel_pid, P_VEL_KP, P_VEL_KI, P_VEL_KD, P_VEL_IMAX);
//-------------------------------------------------------------------------------------------------------------------
void asc_pid_set(asc_pid_t *p, float kp, float ki, float kd, float imax)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->imax = imax;
    asc_pid_reset(p);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清积分、误差历史和增量式的累积输出
// 参数说明     p               PID 实例
// 返回参数     void
// 使用示例     asc_pid_reset(&p_vel_pid);
//-------------------------------------------------------------------------------------------------------------------
void asc_pid_reset(asc_pid_t *p)
{
    p->out_p = p->out_i = p->out_d = p->out = 0;
    p->integrator = 0;
    p->last_error = 0;
    p->last_derivative = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     位置式 PID 的公共计算，hold 为 1 时本拍不累积积分
// 参数说明     p/error/hold    PID 实例、当前误差、是否暂停积分
// 返回参数     float           限幅前的绝对输出
// 使用示例     out = asc_pid_loc_core(p, error, 0);
//-------------------------------------------------------------------------------------------------------------------
static float asc_pid_loc_core(asc_pid_t *p, float error, uint8_t hold)
{
    if (p->ki == 0.0f)
    {
        p->integrator = 0;
    }
    else if (!hold)
    {
        p->integrator += error;
        p->integrator = asc_clamp(p->integrator, -p->imax, p->imax);  // 积分限幅
    }

    p->out_p = p->kp * error;
    p->out_i = p->ki * p->integrator;
    p->out_d = p->kd * (error - p->last_error);
    p->last_error = error;

    p->out = p->out_p + p->out_i + p->out_d;
    return p->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算一拍位置式 PID，只对积分限幅，输出不限幅
// 参数说明     p/error         PID 实例与当前误差
// 返回参数     float           本次绝对输出
// 使用示例     asc_pid_loc_calc(&p_angle_pid, p_vel_pid.out - att.pitch + zero);
//-------------------------------------------------------------------------------------------------------------------
float asc_pid_loc_calc(asc_pid_t *p, float error)
{
    return asc_pid_loc_core(p, error, 0);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算一拍位置式 PID，带输出限幅与条件积分抗饱和
// 参数说明     p/error/low/high PID 实例、当前误差、输出下限与上限
// 返回参数     float           限幅后的输出
// 使用示例     asc_pid_loc_calc_limited(&r_rate_pid, error, -limit, limit);
//-------------------------------------------------------------------------------------------------------------------
float asc_pid_loc_calc_limited(asc_pid_t *p, float error, float low, float high)
{
    // 条件积分：输出已经贴在限幅上、而本拍的积分增量还要往同一个方向推时，本拍不累积。
    // ki*error 才是 out_i 的增量方向 —— kp/ki 为负时(Roll 内环)误差符号是反的，不能只看 error。
    float push = p->ki * error;
    uint8_t hold = (uint8_t)((p->out >= high && push > 0.0f) ||
                         (p->out <= low  && push < 0.0f));

    (void)asc_pid_loc_core(p, error, hold);
    p->out = asc_clamp(p->out, low, high);
    return p->out;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     算一拍带输出限幅的增量式 PID，积分只在不会继续推向饱和时生效
// 参数说明     p/error/low/high PID 实例、当前误差、输出下限与上限
// 返回参数     float           限幅后的累计输出
// 使用示例     asc_pid_inc_calc_limited(&r_rate_pid, error, -limit, limit);
//-------------------------------------------------------------------------------------------------------------------
float asc_pid_inc_calc_limited(asc_pid_t *p, float error, float low, float high)
{
    float error_delta = error - p->last_error;
    float output_without_i;
    float output_candidate;

    p->out_p = p->kp * error_delta;
    p->out_i = p->ki * error;
    p->out_d = p->kd * (error_delta - p->last_derivative);

    output_without_i = p->out + p->out_p + p->out_d;
    output_candidate = output_without_i + p->out_i;
    if ((output_candidate > high && p->out_i > 0.0f) ||
        (output_candidate < low  && p->out_i < 0.0f))
    {
        p->out_i = 0.0f;
    }

    p->last_derivative = error_delta;
    p->last_error = error;
    p->out = asc_clamp(output_without_i + p->out_i, low, high);
    return p->out;
}
