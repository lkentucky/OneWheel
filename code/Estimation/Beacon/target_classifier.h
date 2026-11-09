/*
 * target_classifier.h
 *
 *  Created on: 2025年7月15日
 *      Author: Lenovo
 */



#ifndef TARGET_CLASSIFIER_H_
#define TARGET_CLASSIFIER_H_

#include"image_process.h"

extern Img_process_params img_process_params;

// 主分类函数（输入轮廓数组、输出分类结果）
int classify_targets(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked,                       // 当前追踪目标数(候选集数目)
    ContourInfo* contours,
    int numContours,                        //传入的轮廓数组数目
    ContourInfo* result,
    int maxResult                          //最多候选目标
);

// 主分类函数（输入轮廓数组、输出分类结果）
int classify_targets_pro(
    Time_Small_TrackedTarget* tracked_list,
    int* num_tracked,                       // 当前追踪目标数(候选集数目)
    ContourInfo* contours,
    int numContours,                        //传入的轮廓数组数目
    ContourInfo* result,
    int maxResult                          //最多候选目标
);

#endif /* TARGET_CLASSIFIER_H_ */
