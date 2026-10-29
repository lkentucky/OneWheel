/*********************************************************************************************************************
* CYT2BL3 Opensource Library 即（ CYT2BL3 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT2BL3 开源库的一部分
*
* CYT2BL3 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          brushed_drive
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          IAR 9.40.1
* 适用平台          CYT2BL3
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2026-07-02       pudding             first version
********************************************************************************************************************/

#include "brushed_drive.h"


static float brushed_left_duty_last  = 0.0f;                                    // 左电机上次输出占空比
static float brushed_right_duty_last = 0.0f;                                    // 右电机上次输出占空比


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     有刷电机 H 桥占空比输出
// 参数说明     motor               电机参数结构体指针
// 参数说明     q1q4_open           正转全桥输出函数指针
// 参数说明     q3q2_open           反转全桥输出函数指针
// 参数说明     mos_close           全桥关闭函数指针
// 参数说明     duty_last           上次占空比记录指针
// 返回参数     void
// 使用示例     brushed_apply_duty(&motor_left, left_mos_q1q4_open, left_mos_q3q2_open, left_mos_close, &brushed_left_duty_last);
// 备注信息     有刷模式无堵转保护功能；有刷模式无过载保护功能；有刷模式无转速反馈功能
//-------------------------------------------------------------------------------------------------------------------
static void brushed_apply_duty(motor_struct *motor, void (*q1q4_open)(uint16 duty), void (*q3q2_open)(uint16 duty), void (*mos_close)(void), float *duty_last)
{
    float   duty_now;
    uint16  pwm_duty;

    duty_now = func_limit_ab(motor->motor_duty, -BRUSHED_DUTY_MAX, BRUSHED_DUTY_MAX);   // 限幅最大占空比，保证上桥自举电容充电

    if(motor->motor_protect_state == PROTECT_MODE ||
       (battery_value.protect_flag == 1 && battery_value.battery_state == BATTERY_ERROR))
    {
        if(*duty_last != 0.0f)
        {
            mos_close();
            *duty_last = 0.0f;
        }
        return;
    }

    if(duty_now == *duty_last)
    {
        return;
    }

    *duty_last = duty_now;

    pwm_duty = (uint16)(func_abs(duty_now) * (float)BRUSHED_PWM_PRIOD_LOAD);

    if(pwm_duty == 0)
    {
        mos_close();
    }
    else if(duty_now > 0.0f)
    {
        q1q4_open(pwm_duty);                                                    // A 相 PWM + B 相下桥，正转
    }
    else
    {
        q3q2_open(pwm_duty);                                                    // B 相 PWM + A 相下桥，反转
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左电机 PIT 周期更新
// 参数说明     void
// 返回参数     void
// 使用示例     brushed_left_update_isr();
// 备注信息     由 motor_left_update_isr 在 PIT 中断路径中调用
//-------------------------------------------------------------------------------------------------------------------
void brushed_left_update_isr(void)
{
    brushed_apply_duty(&motor_left, left_mos_q1q4_open, left_mos_q3q2_open, left_mos_close, &brushed_left_duty_last);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     右电机 PIT 周期更新
// 参数说明     void
// 返回参数     void
// 使用示例     brushed_right_update_isr();
// 备注信息     由 motor_right_update_isr 在 PIT 中断路径中调用
//-------------------------------------------------------------------------------------------------------------------
void brushed_right_update_isr(void)
{
    brushed_apply_duty(&motor_right, right_mos_q1q4_open, right_mos_q3q2_open, right_mos_close, &brushed_right_duty_last);
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     双路有刷驱动初始化
// 参数说明     void
// 返回参数     void
// 使用示例     brushed_drive_init();
// 备注信息     PIT 周期更新方式参考无感模式；C 相不参与有刷 H 桥输出
//-------------------------------------------------------------------------------------------------------------------
void brushed_drive_init(void)
{
    motor_left_output_init(BRUSHED_PWM_PRIOD_LOAD, 0);                          // 左电机 PWM 初始化 17 kHz，不启用溢出中断

    motor_right_output_init(BRUSHED_PWM_PRIOD_LOAD, 0);                         // 右电机 PWM 初始化 17 kHz，不启用溢出中断

    left_mos_close();                                                           // 左电机初始关断

    right_mos_close();                                                          // 右电机初始关断

    brushed_left_duty_last  = 0.0f;

    brushed_right_duty_last = 0.0f;

    pit_us_init(LEFT_MOTOR_PIT_TIMER, 50);                                      // 左电机 PIT 周期更新 50us，参考无感模式

    pit_us_init(RIGHT_MOTOR_PIT_TIMER, 50);                                     // 右电机 PIT 周期更新 50us，参考无感模式
}