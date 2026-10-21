/*
 * motor.h
 *
 *  Created on: 2025年3月5日
 *      Author: lenovo
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_
#include "zf_common_headfile.h"

#define MAX_DUTY            (50 )                                               // 最大 MAX_DUTY% 占空比
#define DIR1                (P21_4)
#define PWM1                (ATOM0_CH3_P21_5)

#define ENCODER_1                   (TIM6_ENCODER)
#define ENCODER_1_A                 (TIM6_ENCODER_CH1_P20_3)
#define ENCODER_1_B                 (TIM6_ENCODER_CH2_P20_0)


extern int16 bujin_last_duty;//

extern int16 encoder_data_1;//有刷电机编码器
extern float bujin_car_speed;//行进电机速度
extern float expect_speed;
extern uint8 run_moter;
void Motor_init(void);
void bujin_motor_contron(int duty);
void change_the_speed(void);
#endif /* CODE_MOTOR_H_ */

