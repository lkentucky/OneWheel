#ifndef OTUS_H
#define OTUS_H

#include "zf_common_headfile.h"


/**
 * @brief 计算图像的直方图
 *
 * 计算图像的像素分布，将像素值（0~255）分到指定的直方图数组中
 * 结果将存储到给定的直方图数组中。
 * @param image uint8_t* 输入图像数据
 * @param width int 图像的宽度（像素）
 * @param height int 图像的高度（像素）
 * @param histogram int* 长度为 256 的数组，用于存储图像的直方图
 *                   histogram[i] 表示像素值 i 的出现次数
 * @return void
 */
void calculateHistogram(uint8_t *image,int width,int height,Histogram histogram);

/**
 * @brief 根据直方图检查图像中是否有大于阈值的像素
 *
 * @param histogram int* 直方图数组，长度为 256
 * @param threshold uint8_t 阈值
 * @return 1 存在大于阈值的像素，0 不存在
 */
int hasPixelAboveThresholdInHistogram(Histogram histogram, uint8_t threshold);

/**
 * @brief 使用 Otsu 算法计算最佳阈值
 * 
 * @param histogram 直方图数组，表示图像的像素分布
 * @param width     图像的宽度
 * @param height    图像的高度
 * @param startThreshold   起始阈值，用于计算 Otsu 最优阈值
 * @param otsuresult    存储 Otsu 结果的结构，包括最优阈值和白色区域比例
 * 
 * @return int 1 完成计算，-1 参数错误
 */
int otsuThresholding(const Histogram histogram, int width, int height, uint8_t startThreshold, OtsuResult* otsuresult);

/**
 * @brief 根据阈值将灰度图像转换为二值图像
 * @param input 输入图像
 * @param output 输出二值化图像
 * @param width  图像宽度
 * @param height 图像高度
 * @param threshold 阈值
 */
void gray2binary(uint8_t* input, uint8_t* output,
int width, int height, int threshold);

#endif
