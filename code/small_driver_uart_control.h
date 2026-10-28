#ifndef SMALL_DRIVER_UART_CONTROL_H_
#define SMALL_DRIVER_UART_CONTROL_H_

#include "zf_common_headfile.h"


#define SMALL_DRIVER_UART QCAR_DRIVER_UART

#define SMALL_DRIVER_BAUDRATE QCAR_DRIVER_BAUD

#define SMALL_DRIVER_RX QCAR_DRIVER_RX

#define SMALL_DRIVER_TX QCAR_DRIVER_TX

extern small_device_value_struct motor_value;
extern volatile uint32 small_driver_rx_bytes;
extern volatile uint32 small_driver_speed_frames;
extern volatile uint32 small_driver_feedback_age_ms;
extern volatile uint8 small_driver_feedback_fresh;
extern volatile float small_driver_test_left, small_driver_test_right;



void uart_control_callback(void);                                   // 无刷驱动 串口接收回调函数
void small_driver_tick_1ms(void); /* Always-on key timer; no UART I/O. */

void small_driver_set_duty(int16 left_duty, int16 right_duty);      // 无刷驱动 设置电机占空比

void small_driver_get_speed(void);                                  // 无刷驱动 获取速度信息

/* 65534 arm, 65533 cancel, 65535 metadata, 60000+event, or first row. */
int small_driver_trace_request(unsigned index);
int small_driver_trace_take(char *out, unsigned capacity);
void small_driver_uart_init(void);                                  // 无刷驱动 串口通讯初始化

#endif
