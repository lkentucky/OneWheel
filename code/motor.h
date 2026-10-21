#ifndef _motor_h_
#define _motor_h_
#include "zf_common_headfile.h"

extern int16 real_mid_speed ;

void motor_int(void);
void motor_encoder_test(void);
void motor_ctrl_fly(float f_speed, float l_speed);
void motor_m_ctrl(int speed);
#endif
