 /*
 * balance_control.c
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "balance_control.h"

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
    balance_cascade.cascade_value.gyrp_raw_data=&icm_data_lpf.gyroY;
    balance_cascade.cascade_value.acc_raw_data=&icm_data_lpf.accZ;
    balance_cascade.cascade_value.gyro_ration=4.0f;
    balance_cascade.cascade_value.acc_ration=6.0f;
    balance_cascade.cascade_value.call_cycle=0.005f;
    balance_cascade.cascade_value.mechanical_zero=30;//-40
    balance_cascade.cascade_value.new_mechanical_zero=0;

    balance_cascade.cascade_value.filtering_angle=-balance_cascade.cascade_value.mechanical_zero;
    balance_cascade.cascade_value.angle_temp=-balance_cascade.cascade_value.mechanical_zero;

    balance_cascade.angular_speed_cycle.i_value_max=1000;
    balance_cascade.angular_speed_cycle.i_value_pro=0.1f;
    balance_cascade.angular_speed_cycle.out_max=10000;
    balance_cascade.angle_cycle.i_value_max=4000;
    balance_cascade.angle_cycle.i_value_pro=0.05f;
    balance_cascade.angle_cycle.out_max=8000;
    balance_cascade.speed_cycle.i_value_max=4000;
    balance_cascade.speed_cycle.i_value_pro=0.5f;
    balance_cascade.speed_cycle.out_max=1500;

    balance_cascade.angular_speed_cycle.p=0.0f;
    balance_cascade.angular_speed_cycle.i=0.13f;//0.06
    balance_cascade.angular_speed_cycle.d=0.00f;

    balance_cascade.angle_cycle.p=0.85f;//0.9
    balance_cascade.angle_cycle.i=0.0f;
    balance_cascade.angle_cycle.d=0.08f;

    balance_cascade.speed_cycle.p=0.2f;//0.29
    balance_cascade.speed_cycle.i=0.001f;
    balance_cascade.speed_cycle.d=0.0f;

    //步进环
    balance_cascade.pitch_cascade_value.gyrp_raw_data=&icm_data_lpf.gyroZ;
    balance_cascade.pitch_cascade_value.acc_raw_data=&icm_data_lpf.accY;
    balance_cascade.pitch_cascade_value.gyro_ration=-6.0f;
    balance_cascade.pitch_cascade_value.acc_ration=4.0f;
    balance_cascade.pitch_cascade_value.call_cycle=0.005f;
    balance_cascade.pitch_cascade_value.mechanical_zero=-100;//100
    balance_cascade.pitch_cascade_value.new_mechanical_zero=0;
    balance_cascade.pitch_cascade_value.filtering_angle=-balance_cascade.pitch_cascade_value.mechanical_zero;
    balance_cascade.pitch_cascade_value.angle_temp=-balance_cascade.pitch_cascade_value.mechanical_zero;

    balance_cascade.pitch_angular_speed_cycle.i_value_max=1000;
    balance_cascade.pitch_angular_speed_cycle.i_value_pro=0.1f;
    balance_cascade.pitch_angular_speed_cycle.out_max=10000;
    balance_cascade.pitch_angle_cycle.i_value_max=4000;
    balance_cascade.pitch_angle_cycle.i_value_pro=0.05f;
    balance_cascade.pitch_angle_cycle.out_max=8000;
    balance_cascade.pitch_speed_cycle.i_value_max=4000;
    balance_cascade.pitch_speed_cycle.i_value_pro=0.5f;
    balance_cascade.pitch_speed_cycle.out_max=3500;

    balance_cascade.pitch_angular_speed_cycle.p=0.0f;//0.05
    balance_cascade.pitch_angular_speed_cycle.i=0.023f;//0.02
    balance_cascade.pitch_angular_speed_cycle.d=0.0f;

    balance_cascade.pitch_angle_cycle.p=0.58f;//0.8/0.9
    balance_cascade.pitch_angle_cycle.i=0.0;
    balance_cascade.pitch_angle_cycle.d=0.003f;

    balance_cascade.pitch_speed_cycle.p=5.8f;//0.2/0.9/6.0
    balance_cascade.pitch_speed_cycle.i=0.0f;
    balance_cascade.pitch_speed_cycle.d=0.001f;



    //转向环
    balance_cascade.yaw_cascade_value.gyrp_raw_data=&icm_data_lpf.gyroX;
    balance_cascade.yaw_angular_speed_cycle.i_value_max=1000;
    balance_cascade.yaw_angular_speed_cycle.i_value_pro=0.1f;
    balance_cascade.yaw_angular_speed_cycle.out_max=10000;
    balance_cascade.yaw_erro_cycle.i_value_max=8500;
    balance_cascade.yaw_erro_cycle.i_value_pro=0.05f;
    balance_cascade.yaw_erro_cycle.out_max=8000;

    balance_cascade.yaw_angular_speed_cycle.p=0.62f;   // 增大P以加快响应
    balance_cascade.yaw_angular_speed_cycle.i=0.0f; // 小幅增加I以消除静差
    balance_cascade.yaw_angular_speed_cycle.d=0.0f; // 恢复D至原注释值

    balance_cascade.yaw_erro_cycle.p=9.4f;     // 增大P以提高转向灵敏度
    balance_cascade.yaw_erro_cycle.i=1.55f;     // 减小I以减少积分饱和
    balance_cascade.yaw_erro_cycle.d=0.0f;

    balance_cascade.motor_cycle.i_value_max=5500;
    balance_cascade.motor_cycle.i_value_pro=0.05f;
    balance_cascade.motor_cycle.out_max=8000;
    balance_cascade.motor_cycle.p=11.1f;//9.2
    balance_cascade.motor_cycle.i=0.00f;
    balance_cascade.motor_cycle.d=0.29f;
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
    pid_cycle->out=(int16)(pid_cycle->p*proportion_value+pid_cycle->i*pid_cycle->i_value+pid_cycle->d*differential_value);
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
    if(filter_value->filtering_angle>4000)
    {
        filter_value->filtering_angle=filter_value->mechanical_zero;
    }
}
//----------------------------------------
//函数简介：PID闭环计算（增量式）
//----------------------------------------
void pid_control_incremental(pid_cycle_struct *pid_cycle,float target,float real )
{
    float proportion_value =0;  //比例量
    float differential_value=0; //微分量
    pid_cycle->i_value=target-real;
    differential_value=proportion_value-2*pid_cycle->incremental_data[0]+pid_cycle->incremental_data[1];
    proportion_value=proportion_value-pid_cycle->incremental_data[0];
    pid_cycle->incremental_data[1]=pid_cycle->incremental_data[0];
    pid_cycle->incremental_data[0]=proportion_value;
    pid_cycle->out+=(int16)(pid_cycle->p*proportion_value + pid_cycle->i * pid_cycle->i_value + pid_cycle->d*differential_value);
    pid_cycle->out=func_limit_ab(pid_cycle->out,-pid_cycle->out_max,pid_cycle->out_max);
}
void change_mechanical_zero(void)
{
    balance_cascade.cascade_value.new_mechanical_zero=k1*erro_yawan;//通过k1控制压弯的程度
    balance_cascade.cascade_value.new_mechanical_zero=func_limit_ab(balance_cascade.cascade_value.new_mechanical_zero,-encoder_data_1*k2*erro_yawan,encoder_data_1*k2*erro_yawan);//限幅，人为固定限幅
//    new_mechanical_zero=func_limit_ab(new_mechanical_zero,-90,90);
}
