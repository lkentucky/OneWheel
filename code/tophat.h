/*
 * tophat.h
 *
 *  Created on: 2025年7月9日
 *      Author: Lenovo
 */


/* tophat.h  -------------------------------------------------------- */
#ifndef TOPHAT_H_
#define TOPHAT_H_

#include <stdint.h>

/* 支持任意奇数尺寸的二维核；kernel[k * k] 中 1 表示该位置参与运算 */
void tophat_u8(const uint8_t *src,   /* 原灰度图 */
               uint8_t       *dst,                                             /* 顶帽输出图，须预分配 w*h 字节 */
               int w, int h,
               const uint8_t *kernel,/* k×k 二值核 */
               int k);               /* k 必须是奇数 */

void make_elliptic_kernel(uint8_t *k, int sz);

void process_roi_tophat_thresh(const uint8_t *src, uint8_t *dst);

void process_roi_gradient_thresh(const uint8_t *src, uint8_t *dst);
void process_roi_segment_thresh(const uint8_t *src, uint8_t *dst);
#endif /* TOPHAT_H_ */

