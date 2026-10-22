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
volatile int16 bujin_last_duty = 0;
static int16 requested_duty;
static uint8 output_phase;
static uint32 phase_ticks;
uint8 run_moter=1;
float expect_speed=0;
static uint8 motor_direction_level = MOTOR_DIR_FORWARD_LEVEL;

void Motor_init(void)
{
    encoder_dir_init(ENCODER_1, ENCODER_1_A, ENCODER_1_B);
    /* Single PWM + direction: start with zero duty before enabling DIR GPIO. */
    pwm_init(MOTOR_PWM_PIN, MOTOR_PWM_FREQUENCY, 0);
    pwm_output_enable(MOTOR_PWM_PIN, 0);
    gpio_init(MOTOR_DIR_PIN, GPO, MOTOR_DIR_FORWARD_LEVEL, GPO_PUSH_PULL);
    motor_direction_level = MOTOR_DIR_FORWARD_LEVEL;
    bujin_last_duty = requested_duty = 0;
    output_phase = 0;
}

/* Runtime hardware writes belong only to CPU1's independent output ISR. */
int16 motor_requested_output(void) { return requested_duty; }

void motor_output_service(int16 duty, uint32 now, uint32 ticks_per_ms)
{
    uint8 direction;
    uint32 magnitude;
    uint32 period_ticks = (uint32)(((uint64)ticks_per_ms * 1000u + MOTOR_PWM_FREQUENCY - 1u) / MOTOR_PWM_FREQUENCY);
    uint32 setup_ticks = (uint32)(((uint64)ticks_per_ms * MOTOR_DIR_SETUP_US + 999u) / 1000u);
    if (!run_flag || !run_moter) { duty = 0; }
    if (!duty)
    {
        pwm_output_enable(MOTOR_PWM_PIN, 0);
        pwm_set_duty(MOTOR_PWM_PIN, 0);
        bujin_last_duty = 0;
        output_phase = 0;
        return;
    }
    direction = duty > 0 ? MOTOR_DIR_FORWARD_LEVEL : MOTOR_DIR_REVERSE_LEVEL;
    magnitude = duty > 0 ? (uint32)duty : (uint32)(-(int32)duty);
    if (output_phase == 3 && direction == motor_direction_level)
    {
        pwm_set_duty(MOTOR_PWM_PIN, magnitude);
        bujin_last_duty = duty;
        return;
    }
    if (!output_phase || output_phase == 3 || direction != motor_direction_level && output_phase == 2)
    {
        pwm_output_enable(MOTOR_PWM_PIN, 0);
        pwm_set_duty(MOTOR_PWM_PIN, 0);
        bujin_last_duty = 0;
        phase_ticks = now;
        output_phase = 1;
        return;
    }
    if (output_phase == 1)
    {
        if ((uint32)(now - phase_ticks) < period_ticks) { return; }
        if (direction != motor_direction_level)
        {
            gpio_set_level(MOTOR_DIR_PIN, direction);
            motor_direction_level = direction;
        }
        pwm_set_duty(MOTOR_PWM_PIN, magnitude);
        phase_ticks = now;
        output_phase = 2;
        return;
    }
    if ((uint32)(now - phase_ticks) < (setup_ticks > period_ticks ? setup_ticks : period_ticks)) { return; }
    pwm_set_duty(MOTOR_PWM_PIN, magnitude);
    pwm_output_enable(MOTOR_PWM_PIN, 1);
    output_phase = 3;
    bujin_last_duty = duty;
}

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
    requested_duty = 0;
    if (!run_flag || !run_moter || !(CONTROL_PITCH_ENABLE || CONTROL_MOTOR_ENABLE))
    {
        requested_duty = 0;
        return;
    }
    if (func_abs(balance_cascade.pitch_cascade_value.filtering_angle) > PROTECT_PITCH_RAW ||
        func_abs(balance_cascade.cascade_value.filtering_angle) > PROTECT_ROLL_RAW ||
        func_abs(duty) > PROTECT_TRAVEL_DUTY)
    {
        control_stop();
        return;
    }
    duty *= MOTOR_DIRECTION_SIGN;
    if (duty > 0)
    {
        duty += MOTOR_DEADZONE_DUTY;

    }
    else if (duty < 0)
    {
        duty -= MOTOR_DEADZONE_DUTY;

    }
    else
    {
        requested_duty = 0;
    }
    if (func_abs(duty) > 10000)
    {
        safety_output_trip(SAFETY_FAULT_COMMAND);
        return;
    }
    requested_duty = (int16)duty;
}
