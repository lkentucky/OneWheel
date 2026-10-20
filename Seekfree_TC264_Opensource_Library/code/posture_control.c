/*
 * posture_controal.c
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "posture_control.h"


int16 acc[3];
int16 gyro[3];
ICM_DATA icm_data_lpf;
ICM_DATA re_icm_data;
float pitch_filiter=0;
void imu_data_get(void)
{
    imu660rb_get_acc();
    imu660rb_get_gyro();
    gyro[0] = imu660rb_gyro_x-7.0;    //偏移纠正
    gyro[1] = imu660rb_gyro_y+10;
    gyro[2] = imu660rb_gyro_z-4;
    acc[0]  = imu660rb_acc_x;
    acc[1]  = imu660rb_acc_y;
    acc[2]  = imu660rb_acc_z;
    if(func_abs(gyro[0])<5)
    {
        gyro[0]=0;
    }
    if(func_abs(gyro[1])<5)
    {
        gyro[1]=0;
    }
    if(func_abs(gyro[2])<5)
    {
        gyro[2]=0;
    }


    icm_data_lpf.accX = icm_data_lpf.accX + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[0] - icm_data_lpf.accX)); // 低通滤波
    icm_data_lpf.accY = icm_data_lpf.accY + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[1] - icm_data_lpf.accY));
    icm_data_lpf.accZ = icm_data_lpf.accZ + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)acc[2] - icm_data_lpf.accZ));
    // 低通滤波处理陀螺仪数据
    icm_data_lpf.gyroX = icm_data_lpf.gyroX + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)gyro[0] - icm_data_lpf.gyroX));
    icm_data_lpf.gyroY = icm_data_lpf.gyroY + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3 + balance_cascade.cascade_value.call_cycle)) * ((float)gyro[1] - icm_data_lpf.gyroY));
    icm_data_lpf.gyroZ = icm_data_lpf.gyroZ + ((balance_cascade.cascade_value.call_cycle / (7.9617834e-3+ balance_cascade.cascade_value.call_cycle)) * ((float)gyro[2] - icm_data_lpf.gyroZ));

}
void icm_init(void)
{
    icm_data_lpf.accX = imu660rb_acc_x;
    icm_data_lpf.accY = imu660rb_acc_y;
    icm_data_lpf.accZ = imu660rb_acc_z;
    // 低通滤波处理陀螺仪数据
    icm_data_lpf.gyroX = imu660rb_gyro_x;
    icm_data_lpf.gyroY = imu660rb_gyro_y;
    icm_data_lpf.gyroZ = imu660rb_gyro_z;


    re_icm_data.accX = imu660rb_acc_x;
    re_icm_data.accY = imu660rb_acc_y;
    re_icm_data.accZ = imu660rb_acc_z;
    // 低通滤波处理陀螺仪数据
    re_icm_data.gyroX = imu660rb_gyro_x;
    re_icm_data.gyroY = imu660rb_gyro_y;
    re_icm_data.gyroZ = imu660rb_gyro_z;
}
void dynamic_value_control(void)
{
    int16 left_motor_duty;
    int16 right_motor_duty;
    if(run_flag)
    {
        if(func_abs(balance_cascade.cascade_value.filtering_angle)>2500 ||func_abs(balance_cascade.angular_speed_cycle.out)>10000  )//强制保护
        {
            run_flag=0;
            send_two_floats(balance_cascade.cascade_value.filtering_angle,balance_cascade.angular_speed_cycle.out,1);
            balance_cascade.pitch_angular_speed_cycle.i=0.0f;
            balance_cascade.angular_speed_cycle.i=0.0;
            balance_cascade.yaw_angular_speed_cycle.p=0.0f;
            balance_cascade.angular_speed_cycle.out=0;
        }
//        left_motor_duty=func_limit_ab(-balance_cascade.angular_speed_cycle.out,-4000,4000)+func_limit_ab(balance_cascade.yaw_angular_speed_cycle.out,-6000,6000);//-balance_cascade.angular_speed_cycle.out,-balance_cascade.angle_cycle.out
//        right_motor_duty=func_limit_ab(-balance_cascade.angular_speed_cycle.out,-4000,4000)-func_limit_ab(balance_cascade.yaw_angular_speed_cycle.out,-6000,6000);
        left_motor_duty=-balance_cascade.angular_speed_cycle.out+balance_cascade.yaw_angular_speed_cycle.out;
        right_motor_duty=-balance_cascade.angular_speed_cycle.out-balance_cascade.yaw_angular_speed_cycle.out;
        left_motor_duty=func_limit_ab(left_motor_duty,-10000,10000);
        right_motor_duty=func_limit_ab(right_motor_duty,-10000,10000);
        small_driver_set_duty(left_motor_duty,-right_motor_duty);
    }
    else
    {
        small_driver_set_duty(0,0);
    }
}
void find_ma(void)
{
    system_count++;
    imu_data_get();
    if(system_count%5==0)
    {
        first_order_complementary_filtering(&balance_cascade.cascade_value,*balance_cascade.cascade_value.gyrp_raw_data,*balance_cascade.cascade_value.acc_raw_data);//动量环滤波
        first_order_complementary_filtering(&balance_cascade.pitch_cascade_value,*balance_cascade.pitch_cascade_value.gyrp_raw_data,*balance_cascade.pitch_cascade_value.acc_raw_data);//步进环角速度滤波
    }
}



void pit_isr_callback(void)
{

    system_count++;
    //imu_filiter();
    imu_data_get();// 清空编码器计数
    if(system_count>2000)
    {
        //读入陀螺仪数据

//        erro=retarder(erro,0.3);//误差逐渐累加
        expect_speed=retarder(160,0.03);//160
        if(system_count%20==0)
          {

              small_driver_get_speed();
              encoder_data_1 = encoder_get_count(ENCODER_1);                              // 获取编码器计数
              encoder_clear_count(ENCODER_1);
              car_speed=(motor_value.receive_left_speed_data-motor_value.receive_right_speed_data)/2;
              pid_control (&balance_cascade.speed_cycle,0,car_speed);//速度环
              pid_control (&balance_cascade.pitch_speed_cycle,expect_speed,encoder_data_1);//速度环,步进环-balance_cascade.pitch_speed_cycle.out正反馈
              pid_control(&balance_cascade.motor_cycle,expect_speed,encoder_data_1);
          }
          if(system_count%5==0)
          {
              first_order_complementary_filtering(&balance_cascade.cascade_value,*balance_cascade.cascade_value.gyrp_raw_data,*balance_cascade.cascade_value.acc_raw_data);//动量环滤波
              first_order_complementary_filtering(&balance_cascade.pitch_cascade_value,*balance_cascade.pitch_cascade_value.gyrp_raw_data,*balance_cascade.pitch_cascade_value.acc_raw_data);//-balance_cascade.pitch_angle_cycle.out步进环角速度滤波
//              pitch_filiter=pitch_filiter*(1-0.9)+balance_cascade.pitch_cascade_value.filtering_angle*0.9;
             //压弯是的PID是下面注释的一行，与其区别在于压弯加了一个
             // pid_control (&balance_cascade.angle_cycle,-balance_cascade.speed_cycle.out, balance_cascade.cascade_value.filtering_angle-new_mechanical_zero);//角度环

              pid_control (&balance_cascade.angle_cycle,balance_cascade.speed_cycle.out, balance_cascade.cascade_value.filtering_angle-k1*erro_yawan);//角度环
              pid_control (&balance_cascade.pitch_angle_cycle,balance_cascade.pitch_speed_cycle.out, balance_cascade.pitch_cascade_value.filtering_angle);//角度环
              pid_control(&balance_cascade.yaw_erro_cycle,0,(erro));
          }
          pid_control_incremental (&balance_cascade.angular_speed_cycle,  balance_cascade.angle_cycle.out, *balance_cascade.cascade_value.gyrp_raw_data);//角速度
          pid_control_incremental (&balance_cascade.pitch_angular_speed_cycle,-balance_cascade.pitch_angle_cycle.out, *balance_cascade.pitch_cascade_value.gyrp_raw_data);//角速度环balance_cascade.pitch_angular_speed_cycle.out
          pid_control(&balance_cascade.yaw_angular_speed_cycle,-balance_cascade.yaw_erro_cycle.out,*balance_cascade.yaw_cascade_value.gyrp_raw_data);
//          balance_cascade.pitch_angular_speed_cycle.out= func_limit_ab(balance_cascade.pitch_angular_speed_cycle.out,-3000,3000);
//          balance_cascade.motor_cycle.out=func_limit_ab(balance_cascade.motor_cycle.out,-7000,7000);
          dynamic_value_control();
          bujin_motor_contron(balance_cascade.pitch_angular_speed_cycle.out+balance_cascade.motor_cycle.out);
         // bujin_motor_contron(2000);
    }
}
void imu_filiter(void)
{
    static float a=0.9;
    re_icm_data.gyroX=(float)re_icm_data.gyroX*(1-a)+(float)icm_data_lpf.gyroX*a;
    re_icm_data.gyroY=(float)re_icm_data.gyroY*(1-a)+(float)icm_data_lpf.gyroY*a;
    re_icm_data.gyroZ=(float)re_icm_data.gyroZ*(1-a)+(float)icm_data_lpf.gyroZ*a;
    re_icm_data.accX = (float)re_icm_data.accX*(1-a)+(float)icm_data_lpf.accX*a;
    re_icm_data.accY = (float)re_icm_data.accY*(1-a)+(float)icm_data_lpf.accY*a;
    re_icm_data.accZ = (float)re_icm_data.accZ*(1-a)+(float)icm_data_lpf.accZ*a;
}
