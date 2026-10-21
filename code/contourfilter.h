#ifndef CONTOUR_FILTER
#define CONTOUR_FILTER

#include "zf_common_headfile.h"

/*
 * @brief 计算轮廓与信标灯相对距离
 */
int location_estimate(ContourInfo* contourinfo);


/*
 * @brief  判断一个轮廓是否满足成为信标灯的基本条件
 * @param  contourinfo 轮廓信息 filter_type 分类方法选择 rule_base:基于预先设定好的规则，svm:基于支持向量机分类，mlp:基于神经网络分类
 * @return 1 被分类为信标灯 0 被分类为非信标灯
 */
int ContourFilter(ContourInfo* contourinfo,Filter_Type filter_type);

/*
 * @brief 基于观测经验设置一定规则来进行分类
 */
//int Rule_Base_Filter(ContourInfo* contourinfo);

/*
 * @brief 基于支持向量机的二分类器
 */
//int SVM_Filter(ContourInfo* contourinfo);

/*
 * @brief 基于神经网络的二分类器
 */
float MLP_Filter(ContourInfo* contourinfo);

float MLP_Filter_max(ContourInfo* contourinfo);
#endif
