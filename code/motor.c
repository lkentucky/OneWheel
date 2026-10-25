#include "motor.h"
int16 real_mid_speed;
void motor_int(void)
{
    encoder_dir_init(MOTOR_ENCODER_TIM,MOTOR_ENCODER_COUNT_PIN,MOTOR_ENCODER_DIR_PIN);
    pwm_init(QCAR_MOTOR_PWM,MOTOR_PWM_HZ,0);
    gpio_init(QCAR_MOTOR_DIR,GPO,1,GPO_PUSH_PULL);
}
void motor_m_ctrl(int speed)
{
    if(speed>=0) {pwm_set_duty(QCAR_MOTOR_PWM,speed);gpio_set_level(QCAR_MOTOR_DIR,0);}
    else {pwm_set_duty(QCAR_MOTOR_PWM,-speed);gpio_set_level(QCAR_MOTOR_DIR,1);}
}
