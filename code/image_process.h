#ifndef IMAGE_PROCESS_H
#define IMAGE_PROCESS_H

#include "zf_common_headfile.h"

#define day 1
#define night 2

extern ImageInfo imageinfo;
// 图像行数
//int image_process(ImageInfo* imageinfo);
// 筛选轮廓并更新目标跟踪结果。
//int image_process_multi(ImageInfo* imageinfo);
int image_process_distance(ImageInfo* imageinfo,Turn_info* turn_info);
int image_process_time(ImageInfo* imageinfo, Turn_info* turn_info);
int image_process_time_pro(ImageInfo* imageinfo, Turn_info* turn_info);
// 初始化图像信息。
void initImageInfo(ImageInfo* imageinfo, int width, int height);
void init_img_process_params(void);
void init_img_process_params_2(void);
// 释放图像处理资源。
void freeImageInfo(ImageInfo* imageinfo);

#endif
