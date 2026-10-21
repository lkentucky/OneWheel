#ifndef HOST_HEAD_H
#define HOST_HEAD_H
#include "zf_common_typedef.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#define SPI_3 3
#define P22_2 222
#define P15_8 158
#define P23_1 231
#define MT9V03X_W 188
#define MT9V03X_H 120
#define __dsync() ((void)0)
#include "zf_device_wifi_spi.h"
#include "balance_control.h"
#include "camera.h"
#include "posture_control.h"
extern volatile uint32 system_count;
extern volatile uint8 run_flag;
extern volatile float k1, k2;
extern float expect_speed;
extern int16 encoder_data_1, car_speed, bujin_last_duty;
typedef struct { int16 receive_left_speed_data, receive_right_speed_data; } mock_motor;
extern mock_motor motor_value;
uint8 gpio_get_level(int pin);
void gpio_low(int pin);
void gpio_high(int pin);
void spi_transfer_8bit(int index, const uint8 *tx, uint8 *rx, uint32 length);
void spi_write_8bit_array(int index, const uint8 *tx, uint32 length);
void system_delay_us(uint32 us);
float imu660rb_gyro_transition(int16 raw);
float imu660rb_acc_transition(int16 raw);
#endif
