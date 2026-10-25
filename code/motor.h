#ifndef MOTOR_H
#define MOTOR_H
#include "zf_common_headfile.h"
extern int16 real_mid_speed;
void motor_int(void);
void motor_m_ctrl(int speed);
#endif
