/*
 * mlp.h
 *
 *  Created on: 2025年5月7日
 *      Author: Lenovo
 */

#ifndef CODE_MLP_H_
#define CODE_MLP_H_

// mlp.h
#ifndef MLP_H
#define MLP_H

#include "zf_common_headfile.h"

// —— 网络维度 ——
#define INPUT_SIZE   7
#define H1_SIZE     64
#define H2_SIZE     32
#define OUTPUT_SIZE  1

// —— 参数声明 ——
// 在 mlp.c 中定义并初始化
//小轮廓分类网络
extern const float W1[H1_SIZE][INPUT_SIZE];
extern const float b1[H1_SIZE];
extern const float W2[H2_SIZE][H1_SIZE];
extern const float b2[H2_SIZE];
extern const float W3[OUTPUT_SIZE][H2_SIZE];
extern const float b3[OUTPUT_SIZE];

extern const float MLP_SCALER_MEAN[INPUT_SIZE];
extern const float MLP_SCALER_SCALE[INPUT_SIZE];

//大轮廓分类网络
extern const float W1_max[H1_SIZE][INPUT_SIZE];
extern const float b1_max[H1_SIZE];
extern const float W2_max[H2_SIZE][H1_SIZE];
extern const float b2_max[H2_SIZE];
extern const float W3_max[OUTPUT_SIZE][H2_SIZE];
extern const float b3_max[OUTPUT_SIZE];

extern const float MLP_SCALER_MEAN_max[INPUT_SIZE];
extern const float MLP_SCALER_SCALE_max[INPUT_SIZE];
// —— 激活函数 ——
static inline float relu(float x) {
    return x > 0.0f ? x : 0.0f;
}
static inline float sigmoid(float x) {
    return 1.0f / (1.0f + expf(-x));
}

// —— 前向推理接口 ——
// input: 大小 INPUT_SIZE 的输入向量
// output: 大小 OUTPUT_SIZE 的输出结果
void mlp_forward(const float input[INPUT_SIZE],
                 float output[OUTPUT_SIZE]);

void mlp_forward_max(const float input_raw[INPUT_SIZE],
     float output[OUTPUT_SIZE]);
#endif // MLP_H




#endif /* CODE_MLP_H_ */
