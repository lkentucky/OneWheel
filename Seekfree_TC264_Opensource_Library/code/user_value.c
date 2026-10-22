/*
 * user_value.c
 *
 *  Created on: 2025??3??9??
 *      Author: lenovo
 */
#include "zf_common_headfile.h"
#include "user_value.h"

volatile uint32 system_count;//????????
volatile uint8 run_flag=0;//???§Ò??¦Ë
int16 car_speed; //???§³?????
float count_speed=0;
volatile float k1=CONTROL_K1_DEFAULT;
volatile float k2=CONTROL_K2_DEFAULT;
float insert_value1=0;
float insert_value2=0;
void flash_init(void)
{
    if(flash_check(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX))                      // ?§Ø??????????
    {
        flash_read_page_to_buffer(FLASH_SECTION_INDEX, FLASH_PAGE_INDEX);
        balance_cascade.pitch_angular_speed_cycle.i=flash_union_buffer[0].float_type;
        balance_cascade.pitch_speed_cycle.i=flash_union_buffer[1].float_type;
        balance_cascade.pitch_speed_cycle.d=flash_union_buffer[2].float_type;
    }
}
void send_int16_string(int16_t value) {
    char str_buffer[7]; // ?????? -32768 ?? 32767
    snprintf(str_buffer, sizeof(str_buffer), "%d", value);
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string(str_buffer);
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string("\r\n"); // ???§Ò??????
}

void send_float_string(float value) {
    // Buffer: sign, integer/fraction digits and terminator.
    char str_buffer[20];
    // ??????????????????6¦Ë§³??????????????????
    snprintf(str_buffer, sizeof(str_buffer), "%.6f", value);
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string(str_buffer);
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string("\r\n"); // ????
}

void send_two_floats(float value1, float value2,float value3) {
    char str_buffer[60]; // ????????????????
    // ?????? "value1,value2" ???
    snprintf(str_buffer, sizeof(str_buffer), "%.6f,%.6f,%.6f", value1, value2,value3);
    if (APP_WIFI_ENABLE && IfxCpu_getCoreId() == 1u) { wifi_link_cpu1_log(str_buffer); }
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string(str_buffer);
    if (APP_WIRELESS_UART_ENABLE) wireless_uart_send_string("\r\n"); // ????
}
