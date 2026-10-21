/* fuzzy_control.c：模糊 PID 控制。创建日期：2025年3月8日；作者：yuan。 */
#include "fuzzy_control.h"

//const FuzzySet deltaKp_rules[7][7] =
//{
//        {PB, PM, PM, PS, NM, NM, NB},
//        {PM, PM, PS, PS, NS, NM, NM},
//        {PM, PS, PS, PS, NS, NS, NM},
//        {NB, NB, NB, NB, NB, NB, NB},
//        {NM, NS, NS, PS, PS, PS, PM},
//        {NM, NM, NS, PS, PS, PM, PM},
//        {NB, NM, NM, PS, PM, PM, PB},
//};

const FuzzySet deltaKp_rules[7][7] =
{
        {PB, PB, PM, PM, NM, NB, NB},
        {PB, PM, PM, PS, NM, NM, NB},
        {PM, PM, PS, PS, NS, NM, NM},
        {NB, NM, NM, NS, NM, NM, NB},
        {NM, NM, NS, PS, PS, PM, PM},
        {NB, NM, NM, PS, PM, PM, PB},
        {NB, NB, NM, PM, PM, PB, PB},
};

// Ki 调整量的模糊规则表，需根据实际系统调参。
const FuzzySet deltaKi_rules[7][7] =
{
        {PB, PB, PM, PM, NM, NB, NB},
        {PB, PM, PM, PS, NM, NM, NB},
        {PM, PM, PS, PS, NS, NM, NM},
        {NB, NM, NM, NS, NM, NM, NB},
        {NM, NM, NS, PS, PS, PM, PM},
        {NB, NM, NM, PS, PM, PM, PB},
        {NB, NB, NM, PM, PM, PB, PB},
};

// ΔKd规则（示例）
const FuzzySet deltaKd_rules[7][7] =
{
        {PB, PB, PM, PM, NM, NB, NB},
        {PB, PM, PM, PS, NM, NM, NB},
        {PM, PM, PS, PS, NS, NM, NM},
        {NB, NM, NM, NS, NM, NM, NB},
        {NM, NM, NS, PS, PS, PM, PM},
        {NB, NM, NM, PS, PM, PM, PB},
        {NB, NB, NM, PM, PM, PB, PB},
};

PIDController fw_ang_params;
// PID 参数。
float Kp,Ki,Kd;

/* 模糊隶属度函数。 */
// 计算三角形隶属度。
float triangle_mf(float x, float a, float b, float c)
{
    if (x <= a) return 0;
    else if (a < x && x <= b) return (x - a)/(b - a);
    else if (b < x && x <= c) return (c - x)/(c - b);
    else return 0;
}

// 计算误差 E 在七个模糊集合中的隶属度。
void fuzzify_e(float e, float* membership)
{
    // 归一化到[-1,1]
    float normalized_e = 2*(e - E_MIN)/(E_MAX - E_MIN) - 1;

    // 计算各模糊集合的隶属度。
    membership[NB] = triangle_mf(normalized_e, -1.0, -1.0, -0.5);
    membership[NM] = triangle_mf(normalized_e, -1.0, -0.5, -0.1);
    membership[NS] = triangle_mf(normalized_e, -0.5, -0.1, 0.0);
    membership[ZO] = triangle_mf(normalized_e, -0.1, 0.0, 0.1);
    membership[PS] = triangle_mf(normalized_e, 0.0, 0.1, 0.5);
    membership[PM] = triangle_mf(normalized_e, 0.1, 0.5, 1.0);
    membership[PB] = triangle_mf(normalized_e, 0.5, 1.0, 1.0);
}

// 计算误差变化率 EC 的隶属度。
void fuzzify_ec(float ec, float* membership)
{
    // 归一化到[-1,1]
    float normalized_e = 2*(ec - EC_MIN)/(EC_MAX - EC_MIN) - 1;

    // 计算各模糊集合的隶属度。
    membership[NB] = triangle_mf(normalized_e, -1.0, -1.0, -0.6);
    membership[NM] = triangle_mf(normalized_e, -1.0, -0.6, -0.3);
    membership[NS] = triangle_mf(normalized_e, -0.6, -0.3, 0.0);
    membership[ZO] = triangle_mf(normalized_e, -0.3, 0.0, 0.3);
    membership[PS] = triangle_mf(normalized_e, 0.0, 0.3, 0.6);
    membership[PM] = triangle_mf(normalized_e, 0.3, 0.6, 1.0);
    membership[PB] = triangle_mf(normalized_e, 0.6, 1.0, 1.0);
}

/******************** 解模糊化（针对不同输出变量） ********************/
// 各输出模糊集合的中心值。
float get_center(FuzzySet set, float min, float max)
{
    float interval = (max - min) / 6;
    return min + interval * set; // 依次对应 NB、NM、NS、ZO、PS、PM、PB。
}

// 通用解模糊函数。
float defuzzify(FuzzySet* rules, float* e_mem, float* ec_mem, float min, float max)
{
    float sum = 0.0, weight_sum = 0.0;
    const int num_points = 20;
    float step = (max - min) / (num_points - 1);

    for (int i = 0; i < num_points; i++) {
        float x = min + i * step;
        float mu = 0.0;

        // 计算每条规则的激活程度。
        for (int j = 0; j < 7; j++) {
            for (int k = 0; k < 7; k++) {
                float strength = fmin(e_mem[j], ec_mem[k]);
                FuzzySet out_set = rules[j *7 + k]; // 通过行列索引访问一维规则表。
                float center = get_center(out_set, min, max);

                // 计算该输出集的隶属度
                float mu_rule = triangle_mf(x,
                    (out_set == NB) ? min : get_center(out_set -1, min, max),
                    center,
                    (out_set == PB) ? max : get_center(out_set +1, min, max)
                );
                mu = fmax(mu, fmin(strength, mu_rule));
            }
        }

        sum += x * mu;
        weight_sum += mu;
    }

    return (weight_sum == 0) ? 0 : sum / weight_sum;
}

void pid_compute(PIDController* pid, float angle, float gyro, float dt)
{
    float error = pid->setpoint - angle;
    float ec = gyro;
//    float ec = (error - pid->last_error) / dt;


    // 对误差和误差变化率进行模糊化。
    float e_mem[7], ec_mem[7];
    fuzzify_e(error, e_mem);
    fuzzify_ec(ec, ec_mem);

    // 执行模糊推理和解模糊，得到 PID 参数调整量。
    float delta_Kp = defuzzify((FuzzySet*)deltaKp_rules, e_mem, ec_mem, DKP_MIN, DKP_MAX);
//    float delta_Ki = defuzzify((FuzzySet*)deltaKi_rules, e_mem, ec_mem, DKI_MIN, DKI_MAX);
    float delta_Kd = defuzzify((FuzzySet*)deltaKd_rules, e_mem, ec_mem, DKD_MIN, DKD_MAX);

    Kp += delta_Kp;
//    Ki += delta_Ki;
    Kd += delta_Kd;

    // 参数限幅
    Kp = fmax(fmin(Kp, 60), 30);

    cascade_ang_roll.kp=Kp;
//    Ki = fmax(fmin(Ki, 1), 0.0);
    Kd = fmax(fmin(Kd, 3), 0.8);
    cascade_ang_roll.kd=Kd;
// 历史积分分离逻辑。
//    if (fabs(error) > 10.0) {
//        pid->integral = 0;
//    } else {
//        pid->integral += error * dt;
//    }
//
//    // 计算PID输出
//    float output = Kp * error + Ki * pid->integral + Kd * (-gyro);
//    pid->last_error = error;

}


