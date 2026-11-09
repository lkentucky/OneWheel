#include "contourfilter.h"

#define class_num 8
// 二次多项式参数表
double conic_parameter_table[class_num][5] = {
    {  20,  0.00290, -0.54542, 129.63998, 0 },
    {  30,  0.00289, -0.54307, 120.19978, 0 },
    {  40,  0.00258, -0.48428, 100.88394, 0 },
    {  50,  0.00283, -0.53264,  91.33422, 0 },
    {  70,  0.00200, -0.37515,  68.72991, 0 },
    { 100,  0.00190, -0.35784,  61.49324, 0 },
    { 150,  0.00137, -0.25842,  52.61361, 0 },
    { 200,  0.00070, -0.13189,  34.75724, 0 }
};

// 定义真实位置结构体
typedef struct {
    double distance;  // 估计的距离
    double angle;     // 估计的角度（暂时设为0）
} Real_Relative_Position;

// 估计车辆相对信标灯的真实相对位置（只使用 ContourInfo）
int location_estimate(ContourInfo* contourinfo) {
    // 检查输入参数范围（图像尺寸范围可根据实际修改）
    if (contourinfo->cx < 0 || contourinfo->cx > 188 ||
        contourinfo->cy < 0 || contourinfo->cy > 120) {
        return -1;
    }

    // 计算曲线拟合的预测值（第五列用于临时存储拟合值）
    for (int i = 0; i < class_num; i++) {
        conic_parameter_table[i][4] = conic_parameter_table[i][1] * contourinfo->cx * contourinfo->cx +
                                      conic_parameter_table[i][2] * contourinfo->cx +
                                      conic_parameter_table[i][3];
    }

    // cy 落在最靠前区间外侧
    if (contourinfo->cy > conic_parameter_table[0][4]) {
        contourinfo->distance = 20 -
            (contourinfo->cy - conic_parameter_table[0][4]) / (122 - conic_parameter_table[0][4]) *
            (conic_parameter_table[0][0] - 13);
        return 0;
    }

    // cy 落在两个拟合曲线之间，进行插值
    for (int i = 0; i < class_num - 1; i++) {
        if (contourinfo->cy < conic_parameter_table[i][4] &&
            contourinfo->cy > conic_parameter_table[i + 1][4]) {
            contourinfo->distance = conic_parameter_table[i + 1][0] -
                (contourinfo->cy - conic_parameter_table[i + 1][4]) /
                (conic_parameter_table[i][4] - conic_parameter_table[i + 1][4]) *
                (conic_parameter_table[i + 1][0] - conic_parameter_table[i][0]);
            return 0;
        }
    }

    // cy 落在最远区间外侧
    contourinfo->distance = 400 - (contourinfo->cy) / (conic_parameter_table[class_num - 1][4]) *
                            (400 - conic_parameter_table[class_num - 1][0]);
    return 0;
}


//利用距离与面积的关系进行滤波
int distance_area_filter(ContourInfo* contourinfo, Real_Relative_Position* real_relative_position){
    //当信标灯出现在图像边界时，面积可能被遮挡，暂不判断
    if(contourinfo->cx<1||contourinfo->cx>170)
    {
        return 1;
    }
    //如果cy在30列往上并且面积 为1 直接判断为负类
    if(contourinfo->cy<30&&contourinfo->area<2){
        return 0;
    }
    if(real_relative_position->distance<=30){
       if(contourinfo->area>=700)
           return 1;
       else
           return 0;
    }
    if(real_relative_position->distance>30&&real_relative_position->distance<=40){
          if(contourinfo->area>=500&&contourinfo->area<=1250)
              return 1;
          else
              return 0;
    }
    if(real_relative_position->distance>40&&real_relative_position->distance<=50){
             if(contourinfo->area>=250&&contourinfo->area<=1000)
                 return 1;
             else
                 return 0;
    }
    if(real_relative_position->distance>50&&real_relative_position->distance<=100){
                if(contourinfo->area>=45&&contourinfo->area<=450)
                    return 1;
                else
                    return 0;
    }
    if(real_relative_position->distance>100&&real_relative_position->distance<=150){
                   if(contourinfo->area>=15&&contourinfo->area<=80)
                       return 1;
                   else
                       return 0;
    }
    if(real_relative_position->distance>150){
                      if(contourinfo->area<=50)
                          return 1;
                      else
                          return 0;
    }
    return 0;
}

int Rule_Base_Filter(ContourInfo* contourinfo){
    return 0;
}

int ContourFilter(ContourInfo* contourinfo,Filter_Type filter_type){
    switch (filter_type){
        case rule_base:
            return Rule_Base_Filter(contourinfo);
            break;
        case svm:
            break;
        case mlp:
            break;
        default:
            break;
    }
    return 0;
}


//int SVM_Filter(ContourInfo* contourinfo){
//    //规则1:圆度在0.6-0.9
//    if(contourinfo->boxratio>1||contourinfo->boxratio==0)
//    {
//      set_features(contourinfo->distance,contourinfo->area,contourinfo->ave_gray,contourinfo->circularity,contourinfo->boxratio,imageinfo.best_contour.Hu[0],imageinfo.best_contour.Hu[1]);
//      return get_prediction();
//    }
//    return 0;
//}

float MLP_Filter(ContourInfo* contourinfo){

   float input[INPUT_SIZE] = {
           contourinfo->area,
           contourinfo->cx,
           contourinfo->cy,
           contourinfo->boxratio,
           contourinfo->ave_gray
    };


    float output[OUTPUT_SIZE];
    mlp_forward(input, output);
    return output[0];  // 返回神经网络输出
}

float MLP_Filter_max(ContourInfo* contourinfo){

   float input[INPUT_SIZE] = {
           contourinfo->area,
           contourinfo->cx,
           contourinfo->cy,
           contourinfo->boxratio,
           contourinfo->ave_gray,
           contourinfo->max_gray,
           contourinfo->Std_Varaince
      };


    float output[OUTPUT_SIZE];
    mlp_forward(input, output);
    return output[0];  // 返回神经网络输出
}


