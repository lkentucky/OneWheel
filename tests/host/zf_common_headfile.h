#ifndef HOST_HEAD_H
#define HOST_HEAD_H
#include "zf_common_typedef.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "config.h"
#define SPI_3 3
#define P22_2 222
#define P15_8 158
#define P23_1 231
#define MT9V03X_W 188
#define MT9V03X_H 120
void host_dsync(void);
#define __dsync() host_dsync()
uint32 host_compare_swap(unsigned int *address, uint32 value, uint32 condition);
#define __cmpAndSwap(a,v,c) host_compare_swap((a),(v),(c))
#define DEBUG_UART_INDEX 0
void uart_write_buffer(int index, const uint8 *data, uint32 length);
#include "zf_device_wifi_spi.h"
#include "balance_control.h"
#include "camera.h"
#include "posture_control.h"
#include "app.h"
#define CCU60_CH0 0
#define CCU61_CH0 2
#define UART_3 3
#define UART3_TX_P15_6 156
#define UART3_RX_P15_7 157
#define P21_4 214
#define GPO 1
#define GPO_PUSH_PULL 1
#define ATOM0_CH3_P21_5 215
#define TIM2_ENCODER 2
#define TIM2_ENCODER_CH1_P33_7 337
#define TIM2_ENCODER_CH2_P33_6 336
#define TIM6_ENCODER 6
#define TIM6_ENCODER_CH1_P20_3 203
#define TIM6_ENCODER_CH2_P20_0 200
#define IPS200_TYPE_SPI 0
#define MT9V03X_IMAGE_SIZE (MT9V03X_W * MT9V03X_H)
#define RGB565_RED 1
#define RGB565_GREEN 2
#define TRUE 1
#define KEY_1 0
#define KEY_2 1
#define KEY_3 2
#define KEY_4 3
#define KEY_SHORT_PRESS 1
#include "motor.h"
extern volatile uint32 system_count;
extern volatile uint8 run_flag;
extern volatile float k1, k2;
extern float expect_speed;
extern int16 encoder_data_1, car_speed;
extern volatile int16 bujin_last_duty;
#include "small_driver_uart_control.h"
#include "safety_output.h"
typedef small_device_value_struct mock_motor;
uint8 gpio_get_level(int pin);
void gpio_low(int pin);
void gpio_high(int pin);
void gpio_init(int pin, int direction, int level, int mode);
void gpio_set_level(int pin, int level);
void spi_transfer_8bit(int index, const uint8 *tx, uint8 *rx, uint32 length);
void spi_write_8bit_array(int index, const uint8 *tx, uint32 length);
void system_delay_us(uint32 us);
float imu660rb_gyro_transition(int16 raw);
float imu660rb_acc_transition(int16 raw);
float func_limit_ab(float value, float low, float high);
float func_abs(float value);
void clock_init(void);
void debug_init(void);
void interrupt_global_enable(int priority);
void disable_Watchdog(void);
void Motor_init(void);
void small_driver_uart_init(void);
void small_driver_set_duty(int16 left, int16 right);
void small_driver_get_speed(void);
void pwm_init(int pin, uint32 frequency, uint32 duty);
void pwm_set_duty(int pin, uint32 duty);
void pwm_output_enable(int pin, uint8 enabled);
uint8 uart_try_write_buffer(int index, const uint8 *data, uint32 length);
void uart_init(int index, uint32 baud, int tx, int rx);
void uart_rx_interrupt(int index, uint32 enabled);
uint8 uart_query_byte(int index, uint8 *data);
void encoder_dir_init(int index, int count_pin, int dir_pin);
int16 encoder_get_count(int index);
void encoder_clear_count(int index);
void bujin_motor_contron(int duty);
void key_init(uint32 period);
void key_scanner(void);
int key_get_state(int key);
void key_clear_all_state(void);
void wireless_uart_init(void);
void ips200_init(int interface);
uint8 mt9v03x_init(void);
uint8 imu660rb_init(void);
void imu660rb_get_acc(void);
void imu660rb_get_gyro(void);
void pit_ms_init(int channel, uint32 period);
void system_delay_ms(uint32 ms);
void send_two_floats(float a, float b, float c);
void ips200_show_gray_image(int x,int y,const uint8 *pixels,int w,int h,int dw,int dh,int threshold);
void ips200_draw_point(int x,int y,int color);
void ips200_draw_line(int x,int y,int x2,int y2,int color);
void ips200_show_float(int x,int y,float value,int ints,int decimals);
extern uint8 mt9v03x_finish_flag;
extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];
extern int16 imu660rb_acc_x, imu660rb_acc_y, imu660rb_acc_z;
extern int16 imu660rb_gyro_x, imu660rb_gyro_y, imu660rb_gyro_z;
#include "wifi_link.h"
#endif
