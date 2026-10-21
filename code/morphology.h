#ifndef MORPHOLOGY_H
#define MORPHOLOGY_H

#include "zf_common_headfile.h"

/**
 * @brief 创建椭圆形核
 * @param size 核的大小
 * 
 * @return EllipseKernel 创建的椭圆形核
 */
EllipseKernel createEllipseKernel(int size);

/**
 * @brief 销毁椭圆形核
 * @param kernel 指向椭圆形核的指针
 */
void destroyEllipseKernel(EllipseKernel* kernel);

/**
 * @brief 执行膨胀操作
 * @param input  输入图像
 * @param output 输出膨胀后的图像
 */
void dilate(uint8_t* input, uint8_t* output,
int width, int height, const EllipseKernel* kernel);

void treshold_dilate(uint8_t* input, uint8_t* output, int width, int height, const EllipseKernel* kernel, uint8_t threshold);

#endif
