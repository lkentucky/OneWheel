/*
 * balance_control.h
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */

#ifndef CODE_BALANCE_CONTROL_H_
#define CODE_BALANCE_CONTROL_H_

#include "zf_common_headfile.h"

typedef struct
{
        volatile float d;
        volatile float i;
        float i_value;
        float i_value_max;
        float i_value_pro;
        float incremental_data[2];
        float out;
        float out_max;
        volatile float p;
        float p_value_last;
}pid_cycle_struct;

typedef struct
{
        int16 *gyrp_raw_data;//陀螺仪原始数据
        int16 *acc_raw_data;//角速度原始数据
        float gyro_ration;//一阶互补滤波中的角速度滤波信任程度
        float acc_ration;//一阶互补滤波中的角加速度滤波信任程度
        float call_cycle;//互补滤波函数的调用周期
        float angle_temp;//互补滤波中保存上一次计算值
        float mechanical_zero;//机械零点
        float new_mechanical_zero;
        float filtering_angle;//滤波后的角度
}cascade_common_value_struct;
typedef struct
{
        //俯仰环
        cascade_common_value_struct cascade_value;//串级参数
        pid_cycle_struct angular_speed_cycle;
        pid_cycle_struct angle_cycle;
        pid_cycle_struct speed_cycle;
        //步进环
        cascade_common_value_struct pitch_cascade_value;//串级参数
        pid_cycle_struct pitch_angular_speed_cycle;
        pid_cycle_struct pitch_angle_cycle;
        pid_cycle_struct pitch_speed_cycle;
        //转向环
        cascade_common_value_struct yaw_cascade_value;//串级参数
        pid_cycle_struct yaw_angular_speed_cycle;
        pid_cycle_struct yaw_erro_cycle;
        pid_cycle_struct motor_cycle;



}cascada_value_struct;
extern pid_cycle_struct motor_pid;
extern cascada_value_struct balance_cascade;
extern cascada_value_struct balance_cascade_resave;
extern float new_mechanical_zero;
void balance_cascade_init(void);
void pid_control (pid_cycle_struct *pid_cycle,float target,float real);
void first_order_complementary_filtering(cascade_common_value_struct *filter_value,int16 gyro_raw_data,int16 acc_raw_data);
void pid_control_incremental(pid_cycle_struct *pid_cycle,float target,float real );
void change_mechanical_zero(void);
#endif /* CODE_BALANCE_CONTROL_H_ */
