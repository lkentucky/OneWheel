/*
 * motor.c
 *
 *  Created on: 2025年3月5日
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "motor.h"


float bujin_car_speed;//步进电机速度


int16 encoder_data_1 = 0;
int16 bujin_last_duty = 0;
uint8 run_moter=1;
float expect_speed=200;
void Motor_init(void)
{
//    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                      // 初始化编码器模块与引脚 方向解码编码器模式
//    pwm_init(PWM1, 17000, 0);
    pwm_init(ATOM0_CH2_P21_4,17000,0);
    pwm_init(ATOM0_CH3_P21_5,17000,0);
}

//void Motor_init(void)
//{
//    gpio_init(DIR1, GPO, GPIO_HIGH, GPO_PUSH_PULL);
//    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);                      // 初始化编码器模块与引脚 方向解码编码器模式
//    pwm_init(PWM1, 17000, 0);
////    pwm_init(ATOM0_CH2_P21_4,17000,0);
////    pwm_init(ATOM0_CH3_P21_5,17000,0);
//}

void change_the_speed(void)
{
    if(erro<60)
    {
        expect_speed=300;
    }
    else
    {
        expect_speed=-0.05*erro*erro+6*erro+53;
    }
}
//-------------------
//修改步进轮的pwm
//----------------
void bujin_motor_contron(int duty)
{
	bujin_last_duty = 0;
    if(run_flag)
      {
          if( func_abs(balance_cascade.pitch_cascade_value.filtering_angle)>2000 || func_abs(balance_cascade.cascade_value.filtering_angle)>1500 || func_abs(duty)>8000)//强制保护
          {

              run_flag=0;
              balance_cascade.pitch_angular_speed_cycle.i=0.0f;
              balance_cascade.angular_speed_cycle.i=0.0;
              balance_cascade.yaw_angular_speed_cycle.p=0.0f;
              balance_cascade.angular_speed_cycle.out=0;
          }
          if(duty > 0 && run_moter==1)                                                           // 正转
            {
                duty+=400;
                bujin_last_duty = (int16)duty;
//                gpio_set_level(DIR1, GPIO_HIGH);                                    // DIR输出高电平
//                pwm_set_duty(PWM1, duty );                    // 计算占空比
                pwm_set_duty(ATOM0_CH2_P21_4,0);
                pwm_set_duty(ATOM0_CH3_P21_5,duty);

            }
            else if(duty<0 && run_moter==1)                                                                   // 反转
            {
                duty-=400;
                bujin_last_duty = (int16)duty;
//                gpio_set_level(DIR1, GPIO_LOW);                                     // DIR输出低电平
//                pwm_set_duty(PWM1,-duty);                 // 计算占空比
                pwm_set_duty(ATOM0_CH2_P21_4,-duty);
                pwm_set_duty(ATOM0_CH3_P21_5,0);
            }
            else
            {
                pwm_set_duty(ATOM0_CH2_P21_4,0);
                pwm_set_duty(ATOM0_CH3_P21_5,0);
            }
      }
      else
      {
          pwm_set_duty(ATOM0_CH2_P21_4,0);
          pwm_set_duty(ATOM0_CH3_P21_5,0);
      }
}


//void bujin_motor_contron(int duty)
//{
//    if(run_flag)
//      {
//          if( func_abs(balance_cascade.pitch_cascade_value.filtering_angle)>2000 || func_abs(balance_cascade.cascade_value.filtering_angle)>1500 || func_abs(duty)>8000)//强制保护
//          {
//
//              run_flag=0;
//              balance_cascade.pitch_angular_speed_cycle.i=0.0f;
//              balance_cascade.angular_speed_cycle.i=0.0;
//              balance_cascade.yaw_angular_speed_cycle.p=0.0f;
//              balance_cascade.angular_speed_cycle.out=0;
//          }
//          if(duty > 0 && run_moter==1)                                                           // 正转
//            {
//                duty+=400;
//                gpio_set_level(DIR1, GPIO_HIGH);                                    // DIR输出高电平
//                pwm_set_duty(PWM1, duty );                    // 计算占空比
////                pwm_set_duty(ATOM0_CH2_P21_4,0);
////                pwm_set_duty(ATOM0_CH3_P21_5,duty);
//
//            }
//            else if(duty<0 && run_moter==1)                                                                   // 反转
//            {
//                duty-=400;
//                gpio_set_level(DIR1, GPIO_LOW);                                     // DIR输出低电平
//                pwm_set_duty(PWM1,-duty);                 // 计算占空比
////                pwm_set_duty(ATOM0_CH2_P21_4,-duty);
////                pwm_set_duty(ATOM0_CH3_P21_5,0);
//            }
//            else
//            {
//                gpio_set_level(DIR1, GPIO_LOW);                                     // DIR输出低电平
//                pwm_set_duty(PWM1,0);
//            }
//      }
//      else
//      {
//          gpio_set_level(DIR1, GPIO_LOW);                                     // DIR输出低电平
//          pwm_set_duty(PWM1,0);
//      }
//}

