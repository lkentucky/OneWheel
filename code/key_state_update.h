/*
 * key_state_update.h
 *
 *  Created on: 2025年1月10日
 *      Author: 王屹峰
 */

#ifndef CODE_KEY_STATE_UPDATE_H_
#define CODE_KEY_STATE_UPDATE_H_

#include "zf_common_headfile.h"

extern int key1_count;
extern int key2_count;
extern int key3_count;
extern int key4_count;
extern int key5_count;
extern int EC11_count;
extern float stride;
extern int16 EC11_step;

void stride_update(int key_count);
void key_state_update(void);
void ec11_change_pid(void);
void key_ecll_test(void);
#endif /* CODE_KEY_STATE_UPDATE_H_ */
