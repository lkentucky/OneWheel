/*
 * LQR.h
 *
 *  Created on: 2025年3月21日
 *      Author: yuan
 */

#ifndef CODE_LQR_H_
#define CODE_LQR_H_

#include "zf_common_headfile.h"

#define DT          0.01f    // 控制周期10ms
#define ENCODER_PPR 1024      // 编码器每转脉冲数
#define WHEEL_RADIUS 0.1f     // 车轮半径(m)
#define PULSE_TO_M  (2*M_PI*WHEEL_RADIUS/ENCODER_PPR) // 脉冲转位移系数

/***************************
 * EKF状态估计模块
 ***************************/

extern EKF_State EKF_state;

extern float lqr_control(EKF_State *state);

#endif /* CODE_LQR_H_ */
