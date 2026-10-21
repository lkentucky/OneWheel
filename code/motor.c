#include "motor.h"

int16 real_mid_speed = 0;

void motor_int(void)
{
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);// 初始化中轮方向编码器。
    pwm_init(QCAR_MOTOR_PWM, 17000, 0);// 初始化中轮 PWM：17 kHz，初始占空比为零。
    gpio_init(QCAR_MOTOR_DIR, GPO, 1, GPO_PUSH_PULL);// 初始化中轮方向控制引脚。
}

void motor_ctrl_fly(float f_speed, float l_speed)
{
    if(l_speed >= 0)// 根据速度符号选择另一飞轮旋转方向。
    {
        pwm_set_duty(ATOM0_CH0_P21_2, 10000 - l_speed);
        gpio_set_level(P21_3, 0);
    }
    else
    {
        pwm_set_duty(ATOM0_CH0_P21_2, 10000 + l_speed);
        gpio_set_level(P21_3, 1);
    }

    if(f_speed >= 0)// 根据速度符号选择另一飞轮旋转方向。
    {
        pwm_set_duty(ATOM0_CH2_P21_4, 10000 - f_speed);
        gpio_set_level(P21_5, 1);
    }
    else
    {
        pwm_set_duty(ATOM0_CH2_P21_4, 10000 + f_speed);
        gpio_set_level(P21_5, 0);
    }

}

void motor_m_ctrl(int speed)
{

    if(speed >= 0)
    {
        pwm_set_duty(QCAR_MOTOR_PWM, speed);
        gpio_set_level(QCAR_MOTOR_DIR, 0);
    }
    else
    {
        pwm_set_duty(QCAR_MOTOR_PWM, -speed);
        gpio_set_level(QCAR_MOTOR_DIR, 1);

    }

}

void motor_encoder_test(void)
{

    small_driver_uart_init();
    motor_int();
    /* Keep telemetry on the debug UART; the driver owns its configured UART. */
    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_DEBUG_UART);
    encoder_dir_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0);// 初始化中轮方向编码器。

    float fw_pwm = 0;

    while (TRUE)
      {
        ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 60);
          key_state_update();

          if(key3_count==1)
          {
              key1_count=6;
              key2_count=0;
              key3_count=0;
              fw_pwm=0;
              ips200_clear();
              motor_m_ctrl(0);
              small_driver_set_duty(0,0);
              break;
          }

          if (key1_count==1)
          {
              key1_count=0;
              fw_pwm+=200;
          }

          if (key2_count==1)
          {
              key2_count=0;
              fw_pwm-=200;
          }

          small_driver_set_duty(fw_pwm,fw_pwm);
          motor_m_ctrl(fw_pwm);

          real_mid_speed = encoder_get_count(TIM6_ENCODER);
          encoder_clear_count(TIM6_ENCODER);

          sprintf(txt, "fw_pwm:%.2f",fw_pwm);
          ips200_show_string(40, 120, txt);
          sprintf(txt, "real_mid_speed:%d ",real_mid_speed);
          ips200_show_string(40, 140, txt);
          sprintf(txt, "l_fw_speed:%.2f",motor_value.receive_left_speed_data*0.01f);
          ips200_show_string(40, 160, txt);
          sprintf(txt, "r_fw_speed:%.2f",motor_value.receive_right_speed_data*0.01f);
          ips200_show_string(40, 180, txt);

      }
}
