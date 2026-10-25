#ifndef QCAR_TYPES_H
#define QCAR_TYPES_H
#include <stdint.h>
typedef struct { float x,y,z; } Axis3f;
typedef struct { float balance_angle_x,balance_angle_y; } Mono_params;
typedef struct {
    uint8_t send_data_buffer[7],receive_data_buffer[7];
    uint8_t receive_data_count,sum_check_data;
    int16_t receive_left_speed_data,receive_right_speed_data;
} small_device_value_struct;
#endif
