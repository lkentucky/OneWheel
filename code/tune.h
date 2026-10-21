/*
 * tune.h
 *
 *  Created on: 2025年2月27日
 *      Author: yuan
 */

#ifndef CODE_TUNE_H_
#define CODE_TUNE_H_

#include "zf_common_headfile.h"

extern float data_Get;
extern uint8_t DataBuff[512];
extern uint8_t RxBuffer[1];
extern uint16_t RxLine;

float Get_Data(void);
void USART_PID_Adjust(void);
void set_parameter(float *variable, int channel);
void set_parameter_update(void);
void set_parameter_update_y(void);
void oscilloscope_send_update(void);
void oscilloscope_send_update_y(void);

#endif /* CODE_TUNE_H_ */
