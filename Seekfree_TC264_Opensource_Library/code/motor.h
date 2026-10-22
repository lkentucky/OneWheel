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
#define DIR1                (MOTOR_DIR_PIN)
#define PWM1                (MOTOR_PWM_PIN)

#define ENCODER_1                   (MOTOR_ENCODER_INDEX)
#define ENCODER_1_A                 (MOTOR_ENCODER_COUNT_PIN)
#define ENCODER_1_B                 (MOTOR_ENCODER_DIRECTION_PIN)


extern volatile int16 bujin_last_duty;//

extern int16 encoder_data_1;//有刷电机编码器
extern float bujin_car_speed;//行进电机速度
extern float expect_speed;
extern uint8 run_moter;
void Motor_init(void);
int16 motor_requested_output(void);
void motor_output_service(int16 duty, uint32 now_ticks, uint32 ticks_per_ms);
void bujin_motor_contron(int duty);
void change_the_speed(void);
#endif /* CODE_MOTOR_H_ */

