 /*
 * balance_control.c
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "balance_control.h"
#include <float.h>

cascada_value_struct balance_cascade;
cascada_value_struct balance_cascade_resave;
pid_cycle_struct motor_pid;
float new_mechanical_zero=0;

//****************************
//函数简介 串级平衡算法初始化
//返回参数 void
//使用实例 balance_cascade_init();
//*********************************

void balance_cascade_init(void)
{
    balance_cascade.cascade_value.gyrp_raw_data=&ROLL_GYRO_RAW;
    balance_cascade.cascade_value.acc_raw_data=&ROLL_ACCEL_RAW;
    balance_cascade.cascade_value.gyro_ration=ROLL_FILTER_GYRO_WEIGHT;
    balance_cascade.cascade_value.acc_ration=ROLL_FILTER_ACCEL_WEIGHT;
    balance_cascade.cascade_value.call_cycle=CONTROL_FILTER_DT_S;
    balance_cascade.cascade_value.mechanical_zero=ROLL_MECHANICAL_ZERO;//-40
    balance_cascade.cascade_value.new_mechanical_zero=0;

    balance_cascade.cascade_value.filtering_angle=-balance_cascade.cascade_value.mechanical_zero;
    balance_cascade.cascade_value.angle_temp=-balance_cascade.cascade_value.mechanical_zero;

    balance_cascade.angular_speed_cycle.i_value_max=PID_ROLL_RATE_I_LIMIT;
    balance_cascade.angular_speed_cycle.i_value_pro=PID_ROLL_RATE_I_SCALE;
    balance_cascade.angular_speed_cycle.out_max=PID_ROLL_RATE_OUT_LIMIT;
    balance_cascade.angle_cycle.i_value_max=PID_ROLL_ANGLE_I_LIMIT;
    balance_cascade.angle_cycle.i_value_pro=PID_ROLL_ANGLE_I_SCALE;
    balance_cascade.angle_cycle.out_max=PID_ROLL_ANGLE_OUT_LIMIT;
    balance_cascade.speed_cycle.i_value_max=PID_ROLL_SPEED_I_LIMIT;
    balance_cascade.speed_cycle.i_value_pro=PID_ROLL_SPEED_I_SCALE;
    balance_cascade.speed_cycle.out_max=PID_ROLL_SPEED_OUT_LIMIT;

    balance_cascade.angular_speed_cycle.p=PID_ROLL_RATE_P;
    balance_cascade.angular_speed_cycle.i=PID_ROLL_RATE_I;//0.06
    balance_cascade.angular_speed_cycle.d=PID_ROLL_RATE_D;

    balance_cascade.angle_cycle.p=PID_ROLL_ANGLE_P;//0.9
    balance_cascade.angle_cycle.i=PID_ROLL_ANGLE_I;
    balance_cascade.angle_cycle.d=PID_ROLL_ANGLE_D;

    balance_cascade.speed_cycle.p=PID_ROLL_SPEED_P;//0.29
    balance_cascade.speed_cycle.i=PID_ROLL_SPEED_I;
    balance_cascade.speed_cycle.d=PID_ROLL_SPEED_D;

    //步进环
    balance_cascade.pitch_cascade_value.gyrp_raw_data=&PITCH_GYRO_RAW;
    balance_cascade.pitch_cascade_value.acc_raw_data=&PITCH_ACCEL_RAW;
    balance_cascade.pitch_cascade_value.gyro_ration=PITCH_FILTER_GYRO_WEIGHT;
    balance_cascade.pitch_cascade_value.acc_ration=PITCH_FILTER_ACCEL_WEIGHT;
    balance_cascade.pitch_cascade_value.call_cycle=CONTROL_FILTER_DT_S;
    balance_cascade.pitch_cascade_value.mechanical_zero=PITCH_MECHANICAL_ZERO;//100
    balance_cascade.pitch_cascade_value.new_mechanical_zero=0;
    balance_cascade.pitch_cascade_value.filtering_angle=-balance_cascade.pitch_cascade_value.mechanical_zero;
    balance_cascade.pitch_cascade_value.angle_temp=-balance_cascade.pitch_cascade_value.mechanical_zero;

    balance_cascade.pitch_angular_speed_cycle.i_value_max=PID_PITCH_RATE_I_LIMIT;
    balance_cascade.pitch_angular_speed_cycle.i_value_pro=PID_PITCH_RATE_I_SCALE;
    balance_cascade.pitch_angular_speed_cycle.out_max=PID_PITCH_RATE_OUT_LIMIT;
    balance_cascade.pitch_angle_cycle.i_value_max=PID_PITCH_ANGLE_I_LIMIT;
    balance_cascade.pitch_angle_cycle.i_value_pro=PID_PITCH_ANGLE_I_SCALE;
    balance_cascade.pitch_angle_cycle.out_max=PID_PITCH_ANGLE_OUT_LIMIT;
    balance_cascade.pitch_speed_cycle.i_value_max=PID_PITCH_SPEED_I_LIMIT;
    balance_cascade.pitch_speed_cycle.i_value_pro=PID_PITCH_SPEED_I_SCALE;
    balance_cascade.pitch_speed_cycle.out_max=PID_PITCH_SPEED_OUT_LIMIT;

    balance_cascade.pitch_angular_speed_cycle.p=PID_PITCH_RATE_P;//0.05
    balance_cascade.pitch_angular_speed_cycle.i=PID_PITCH_RATE_I;//0.02
    balance_cascade.pitch_angular_speed_cycle.d=PID_PITCH_RATE_D;

    balance_cascade.pitch_angle_cycle.p=PID_PITCH_ANGLE_P;//0.8/0.9
    balance_cascade.pitch_angle_cycle.i=PID_PITCH_ANGLE_I;
    balance_cascade.pitch_angle_cycle.d=PID_PITCH_ANGLE_D;

    balance_cascade.pitch_speed_cycle.p=PID_PITCH_SPEED_P;//0.2/0.9/6.0
    balance_cascade.pitch_speed_cycle.i=PID_PITCH_SPEED_I;
    balance_cascade.pitch_speed_cycle.d=PID_PITCH_SPEED_D;



    //转向环
    balance_cascade.yaw_cascade_value.gyrp_raw_data=&YAW_GYRO_RAW;
    balance_cascade.yaw_angular_speed_cycle.i_value_max=PID_YAW_RATE_I_LIMIT;
    balance_cascade.yaw_angular_speed_cycle.i_value_pro=PID_YAW_RATE_I_SCALE;
    balance_cascade.yaw_angular_speed_cycle.out_max=PID_YAW_RATE_OUT_LIMIT;
    balance_cascade.yaw_erro_cycle.i_value_max=PID_YAW_I_LIMIT;
    balance_cascade.yaw_erro_cycle.i_value_pro=PID_YAW_I_SCALE;
    balance_cascade.yaw_erro_cycle.out_max=PID_YAW_OUT_LIMIT;

    balance_cascade.yaw_angular_speed_cycle.p=PID_YAW_RATE_P;   // 增大P以加快响应
    balance_cascade.yaw_angular_speed_cycle.i=PID_YAW_RATE_I; // 小幅增加I以消除静差
    balance_cascade.yaw_angular_speed_cycle.d=PID_YAW_RATE_D; // 恢复D至原注释值

    balance_cascade.yaw_erro_cycle.p=PID_YAW_P;     // 增大P以提高转向灵敏度
    balance_cascade.yaw_erro_cycle.i=PID_YAW_I;     // 减小I以减少积分饱和
    balance_cascade.yaw_erro_cycle.d=PID_YAW_D;

    balance_cascade.motor_cycle.i_value_max=PID_MOTOR_I_LIMIT;
    balance_cascade.motor_cycle.i_value_pro=PID_MOTOR_I_SCALE;
    balance_cascade.motor_cycle.out_max=PID_MOTOR_OUT_LIMIT;
    balance_cascade.motor_cycle.p=PID_MOTOR_P;//9.2
    balance_cascade.motor_cycle.i=PID_MOTOR_I;
    balance_cascade.motor_cycle.d=PID_MOTOR_D;
    memcpy(&balance_cascade_resave,&balance_cascade,sizeof(balance_cascade_resave));
}

//**************************************************
//函数简介 PID闭环计算（位置式）
//参数说明 pid_cycle PID参数结构体
//参数说明 target 目标值
//参数说明 real 当前值
//使用实例  pid_control (&balance_cascade.speed_cycle,0,motor.encoder_data)
//***************************************************
void pid_control (pid_cycle_struct *pid_cycle,float target,float real)
{
    float proportion_value =0;  //比例量
    float differential_value=0; //微分量
    proportion_value=target-real;
    pid_cycle->i_value+=(proportion_value*pid_cycle->i_value_pro); //积分量=积分量+积分程度
    pid_cycle->i_value=func_limit_ab(pid_cycle->i_value,-pid_cycle->i_value_max,pid_cycle->i_value_max);

    differential_value=proportion_value-pid_cycle->p_value_last;
    pid_cycle->out=(pid_cycle->p*proportion_value+pid_cycle->i*pid_cycle->i_value+pid_cycle->d*differential_value);
    pid_cycle->out=func_limit_ab(pid_cycle->out,-pid_cycle->out_max,pid_cycle->out_max);
    pid_cycle->p_value_last=proportion_value;
}

//-------------------------------------------
//函数简介 一阶互补滤波
//参数说明 filter_value 滤波参数结构体
//参数说明 gyro_raw_data 陀螺仪原始数据
//参数说明 acc_raw_data 加速度原始数据
//使用实例first_order_complementary_filtering(&balance_cascade.cascade_value,*balance_cascade.cascade_value.gyrp_raw_data,*balance_cascade.cascade_value.acc_raw_data);

//____________________________________________
void first_order_complementary_filtering(cascade_common_value_struct *filter_value,int16 gyro_raw_data,int16 acc_raw_data)
{
    float gryo_temp; //角速度计算临时变量
    float acc_temp;  //加速度计算临时变量
    gryo_temp=gyro_raw_data*filter_value->gyro_ration;//角速度数据*角速度置信度（一般为4）
    acc_temp=(acc_raw_data-filter_value->angle_temp)*filter_value->acc_ration;
    filter_value->angle_temp+=((gryo_temp+acc_temp)*filter_value->call_cycle);
    filter_value->filtering_angle=filter_value->angle_temp+filter_value->mechanical_zero;
    if (!(filter_value->angle_temp >= -FLT_MAX && filter_value->angle_temp <= FLT_MAX) ||
        !(filter_value->filtering_angle >= -FILTER_STATE_LIMIT_RAW &&
          filter_value->filtering_angle <= FILTER_STATE_LIMIT_RAW))
    {
        float fault_value = filter_value->filtering_angle;
        safety_output_trip(SAFETY_FAULT_FILTER);
        /* Preserve a visibly unsafe signed result; never replace by the zero. */
        filter_value->filtering_angle = fault_value < 0.0f ? -FILTER_STATE_LIMIT_RAW : FILTER_STATE_LIMIT_RAW;
        filter_value->angle_temp = filter_value->filtering_angle - filter_value->mechanical_zero;
    }
}
//----------------------------------------
//函数简介：PID闭环计算（增量式）
//----------------------------------------
void pid_control_incremental(pid_cycle_struct *pid_cycle,float target,float real )
{
    float error = target - real;
    float delta_p = error - pid_cycle->incremental_data[0];
    float delta_d = error - 2.0f * pid_cycle->incremental_data[0] + pid_cycle->incremental_data[1];
    pid_cycle->i_value = error;
    pid_cycle->out += pid_cycle->p * delta_p + pid_cycle->i * error + pid_cycle->d * delta_d;
    pid_cycle->out = func_limit_ab(pid_cycle->out, -pid_cycle->out_max, pid_cycle->out_max);
    pid_cycle->incremental_data[1] = pid_cycle->incremental_data[0];
    pid_cycle->incremental_data[0] = error;
}
void change_mechanical_zero(void)
{
    balance_cascade.cascade_value.new_mechanical_zero=k1*erro_yawan;//通过k1控制压弯的程度
    balance_cascade.cascade_value.new_mechanical_zero=func_limit_ab(balance_cascade.cascade_value.new_mechanical_zero,-encoder_data_1*k2*erro_yawan,encoder_data_1*k2*erro_yawan);//限幅，人为固定限幅
//    new_mechanical_zero=func_limit_ab(new_mechanical_zero,-90,90);
}

void balance_cascade_reset(void)
{
    pid_cycle_struct *loops[] = {
        &balance_cascade.angle_cycle, &balance_cascade.speed_cycle,
        &balance_cascade.angular_speed_cycle, &balance_cascade.pitch_angle_cycle,
        &balance_cascade.pitch_speed_cycle, &balance_cascade.pitch_angular_speed_cycle,
        &balance_cascade.motor_cycle, &balance_cascade.yaw_erro_cycle,
        &balance_cascade.yaw_angular_speed_cycle
    };
    uint8 index;
    for (index = 0; index < sizeof(loops) / sizeof(loops[0]); index++)
    {
        loops[index]->i_value = 0;
        loops[index]->p_value_last = 0;
        loops[index]->incremental_data[0] = 0;
        loops[index]->incremental_data[1] = 0;
        loops[index]->out = 0;
    }
}
