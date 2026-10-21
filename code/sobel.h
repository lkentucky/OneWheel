/*
 * sobel.h
 *
 *  Created on: 2025年5月6日
 *      Author: Lenovo
 */

#ifndef CODE_SOBEL_H_
#define CODE_SOBEL_H_


#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

/**
 * four_sobel_binarize
 *   对 input 图像做四向 Sobel 边缘检测并二值化，结果写入 output。
 *
 * @param in   指向输入灰度图像的指针，长度应为 WIDTH*HEIGHT
 * @param out  指向输出二值图像的指针，长度应为 WIDTH*HEIGHT
 * @param threshold 二值化参数
 */
void four_sobel_binarize(const uint8_t *in, uint8_t *out,uint8_t threshold,int WIDTH,int HEIGHT);



#endif /* CODE_SOBEL_H_ */
