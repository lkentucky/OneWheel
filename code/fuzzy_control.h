/* fuzzy_control.h：模糊 PID 接口。创建日期：2025年3月8日；作者：yuan。 */

#ifndef CODE_FUZZY_CONTROL_H_
#define CODE_FUZZY_CONTROL_H_

#include "stdio.h"
#include "math.h"
#include "zf_common_headfile.h"

/******************** 模糊PID参数定义 ********************/
// 模糊集合：NB 负大、NM 负中、NS 负小、ZO 零、PS 正小、PM 正中、PB 正大。

// 误差 E 和误差变化率 EC 的论域范围。
#define E_MIN   -10.0    // 倾角误差范围：-30 度至 +30 度。
#define E_MAX    10.0
#define EC_MIN  -1.0    // 误差变化率范围：-10 至 +10，具体单位与调用方输入保持一致。
#define EC_MAX   1.0

// 模糊输出：ΔKp, ΔKi, ΔKd的调整量范围
#define DKP_MIN  -5
#define DKP_MAX   5
#define DKI_MIN  -0.1
#define DKI_MAX   0.1
#define DKD_MIN  -0.15
#define DKD_MAX   0.15

extern PIDController fw_ang_params;
extern float Kp;
extern float Ki;
extern float Kd;

float triangle_mf(float x, float a, float b, float c);
void fuzzify_e(float e, float* membership);
void fuzzify_ec(float ec, float* membership);
float get_center(FuzzySet set, float min, float max);
float defuzzify(FuzzySet* rules, float* e_mem, float* ec_mem, float min, float max);
void pid_compute(PIDController* pid, float angle, float gyro, float dt);

#endif /* CODE_FUZZY_CONTROL_H_ */
