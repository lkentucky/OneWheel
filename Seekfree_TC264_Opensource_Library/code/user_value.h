/*
 * user_value.h
 *
 *  Created on: 2025年3月9日
 *      Author: lenovo
 */

#ifndef CODE_USER_VALUE_H_
#define CODE_USER_VALUE_H_

#include "zf_common_headfile.h"

#define FLASH_SECTION_INDEX       (0)                                 // 存储数据用的扇区
#define FLASH_PAGE_INDEX          (8)
extern volatile uint32 system_count;//系统时间计数
extern volatile uint8 run_flag;//运行标志位
extern int16 car_speed; //当前小车速度
extern volatile float k1;
extern volatile float k2;
extern float count_speed;
extern float insert_value1;
extern float insert_value2;
void flash_init(void);
void send_int16_string(int16_t value);
void send_float_string(float value);
void send_two_floats(float value1, float value2,float value3);
#endif /* CODE_USER_VALUE_H_ */
