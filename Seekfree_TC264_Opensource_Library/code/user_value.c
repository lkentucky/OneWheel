/*
 * user_value.c
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "user_value.h"

uint32 system_count;//系统时间计数
uint8 run_flag=1;//运行标志位
int16 car_speed; //当前小车速度
float count_speed=0;
float k1=1.25;
float k2=0.5;
float insert_value1=0;
float insert_value2=0;
void flash_init(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // 判断是否有数据
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        balance_cascade.pitch_angular_speed_cycle.i=flash_union_buffer[0].float_type;
        balance_cascade.pitch_speed_cycle.i=flash_union_buffer[1].float_type;
        balance_cascade.pitch_speed_cycle.d=flash_union_buffer[2].float_type;
    }
}
void send_int16_string(int16_t value) {
    char str_buffer[7]; // 足够容纳 -32768 到 32767
    snprintf(str_buffer, sizeof(str_buffer), "%d", value);
    wireless_uart_send_string(str_buffer);
    wireless_uart_send_string("\r\n"); // 换行便于解析
}

void send_float_string(float value) {
    // 缓冲区大小：符号(1) + 整数部分(10) + 小数点(1) + 小数部分(6) + 结束符(1) = 19
    char str_buffer[20];
    // 格式化为浮点数，保留6位小数（可根据需求调整）
    snprintf(str_buffer, sizeof(str_buffer), "%.6f", value);
    wireless_uart_send_string(str_buffer);
    wireless_uart_send_string("\r\n"); // 换行
}

void send_two_floats(float value1, float value2,float value3) {
    char str_buffer[60]; // 足够容纳两个浮点数
    // 格式化为 "value1,value2" 格式
    snprintf(str_buffer, sizeof(str_buffer), "%.6f,%.6f,%.6f", value1, value2,value3);
    wireless_uart_send_string(str_buffer);
    wireless_uart_send_string("\r\n"); // 换行
}
