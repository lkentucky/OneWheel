#ifndef QCAR_TYPES_H
#define QCAR_TYPES_H
#define WINDOW_SIZE 5
#define MAX_BEACONS 3
#define QCAR_MAX_CONTOURS 32
typedef int Histogram[256];

// 定义 Otsu 算法结果结构
typedef struct {
    uint8_t startThreshold;    // 起始阈值
    uint8_t optimalThreshold;  // 最优阈值
    double whiteAreaRatio;     // 白色区域比例
} OtsuResult;

//—————— 数据结构定义 ——————//
typedef struct {
    int16_t row;
    int16_t col;
} Point;

// 定义椭圆形核
typedef struct {
    int size;         // 核的大小 (例如 2x2, 3x3)
    uint8_t* kernel;  // 存储椭圆形核的内存
} EllipseKernel;

typedef struct{
    int x,y,w,h;
}Bbox;

typedef struct {
    Point *points;  // 按边界顺序排列的轮廓点
    int count;      // 点数
} Contour;


#define WIN_LEN 40
////维护被激活的区域
//typedef struct {
//    Bbox bbox;
//    int miss;              // 连续未命中帧数
//
//    float std_q[WIN_LEN];      // 方差滑动窗口
//    uint8_t gray_q[WIN_LEN];     // 最大灰度滑动窗口
//    uint8_t idx_q;         // 环形队列索引（共享）
//
//    float max_std;       // 历史最大方差
//    uint8_t max_gray;      // 历史最大灰度
//} Time_Small_TrackedTarget;

typedef struct {
    Bbox bbox;
    int miss;              // 连续未命中帧数

    float std_q[WIN_LEN];      // 方差滑动窗口
    uint8_t gray_q[WIN_LEN];     // 最大灰度滑动窗口
    float mean_gray[WIN_LEN];  // 平均灰度滑动窗口
    uint8_t idx_q;         // 环形队列索引（共享）

    float max_std;       // 历史最大方差
    uint8_t max_gray;      // 历史最大灰度
} Time_Small_TrackedTarget;

// 轮廓信息结构体，用于存储轮廓的圆度、质心和面积,外接矩形长宽比，轮廓平均灰度
typedef struct {
    //float circularity;
    float cx, cy;
    float x,y;
    float area;
    float boxratio;
    float ave_gray;
    //int   conner_point_num;
    //float Hu[3];
    float Std_Varaince;//标准差
    //uint16_t range;//灰度变化范围
    //uint16_t middle_range;//灰度中位数
    Bbox bbox;
    float max_gray;
    float distance;//距离计算
    float angle;
    float prob;//置信度
    float decision;
    float contrast_ratio;
    float mingray;
    int w;
    int h;
} ContourInfo;

typedef struct{
  uint8_t* image;        // 图像数据
//  uint8_t dilateimage[22560];
  uint8_t dayflag;       //白天使用方法or晚上使用
  uint8_t binaryimage[120][188];  // 二值化图像
  uint8_t startthreshold; // 起始阈值，用于图像分割
  int whiteRatiothreshold; // 白色区域的比例阈值
  uint8_t maxPixelthreshold; // 最大像素值阈值
  Histogram histogram; // 图像的直方图
  OtsuResult otusresult; // Otsu算法的结果
  Point last_best_point; // 上一个最佳点的坐标
  Point best_point; // 当前最佳点的坐标
  int islostframe; // 是否丢失帧
  int lostframe; // 丢失的帧数
  int maxlostfram; // 最大丢失帧数
  int isfirstframe; // 是否为第一帧
  int totalContours;
  int numContours; // 轮廓的数量
  int last_numContours; // 轮廓的数量
  int filterContours;//满足要求的所有轮廓
  float mincircularity; // 最小圆度
  float maxcircularity; // 最大圆度
  EllipseKernel minsizekernel; // 最小膨胀核
  EllipseKernel maxsizekernel; // 最大膨胀核
  int width; // 图像的宽度
  int height; // 图像的高度
  int sobel_threshold;//sobel二值化参数
  uint8_t contournum_change;//信标灯数目是否发生变化
  int time_num;
  ContourInfo contourinfo[QCAR_MAX_CONTOURS]; // 存储轮廓信息的数组
  ContourInfo contour_filter[QCAR_MAX_CONTOURS];//有效轮廓
  ContourInfo contour_filter_sort[QCAR_MAX_CONTOURS];//排序后的有效轮廓
  Time_Small_TrackedTarget time_small_track_target[QCAR_MAX_CONTOURS];//时间追踪信息
  ContourInfo best_contour;//当前最佳轮廓
  ContourInfo last_best_contour;//上一次最佳轮廓
  uint8_t minrow;
  float fixed_threshold;
  float domain_radius;
  float std_var_threshold;
  float boxratio_threshold;
  float far_contour_max_area;
  float near_contour_max_area;
  float near_contour_min_area;
  float near_threshold;
  float far_threshold;
}ImageInfo;

//分类器类型
typedef enum{
    rule_base,
    svm,
    mlp
}Filter_Type;

typedef struct {
    float x;
    float y;
    float z;
} Axis3f;

typedef enum { NB, NM, NS, ZO, PS, PM, PB } FuzzySet;

typedef struct
{
    float setpoint;
    float last_error;
    float integral;
} PIDController;

typedef struct
{
   float X;    //x轴零漂
   float Y;    //y轴零漂
   float Z;    //z轴零漂

}Offset;

typedef struct {
    float x;       // 位移估计(m)
    float v;       // 速度(m/s)
    float theta;   // 倾斜角(rad)
    float omega;   // 角速度(rad/s)
    float P[4][4]; // 误差协方差矩阵
} EKF_State;

typedef struct {
    float buffer[WINDOW_SIZE];
    int index;
    float sum;
} MovingAverage;

typedef struct
{
        float   kp;     //  p
        float   ykp;    // 附加比例系数。
        float   ki;     //  i
        float   kd;     //  d
        float   gyro_kd;//D
        float   imax;   // 积分限幅。


        float   out_p;  // 比例项输出。
        float   out_yp; // 角速度反馈的微分系数。
        float   out_i;  // 积分项输出。
        float   out_d;  // 微分项输出。
        float   out_gyro_d;// 角速度反馈微分项输出。
        float   out;        // PID 总输出。

        float   integrator;// 累计积分值。
        float   last_error;// 上一次误差。
        float   derivative;// 误差变化量。
        float   last_derivative;// 上一次误差变化量。
        unsigned  long      last_t;     // 上一次更新时间。

}pid_param_t;

typedef struct
{
        float setspeed;
        float fast_speed;
        float slow_speed;
        float balance_angle_x;
        float balance_angle_y;
        float dead_zone;
        float dead_zone_p;
        float turn_k1_l;
        float turn_k2_l;
        float turn_k1_r;
        float turn_k2_r;
        float turn_k1_p_l;
        float turn_k1_p_r;
        float turn_k2_p_l;
        float turn_k2_p_r;
        float speed_flag;
}Mono_params;

typedef struct
{
       float p;
       float i;
       float d;
} pid;

typedef struct
{
    uint8 send_data_buffer[7];                  // 发送缓冲数组

    uint8 receive_data_buffer[7];               // 接收缓冲数组

    uint8 receive_data_count;                   // 接收计数

    uint8 sum_check_data;                       // 校验位

    int16 receive_left_speed_data;              // 接收到的左侧电机速度数据

    int16 receive_right_speed_data;             // 接收到的右侧电机速度数据

}small_device_value_struct;

typedef struct {
    int id;
    float r;       // 信标角度，单位与调用方保持一致。
    float theta;
    float x;
    float y;
    float cx;
    float cy;
} Beacon;

typedef struct {
    float total_dist;
    float total_angle;
    float first_cx_diff;
    int order[MAX_BEACONS];
} Path;

typedef struct {
    int next_beacon;
    int last_turn_flag;
    int turn_flag;
    int track_flag;
    float cx;
    float cy;
    float weight_dist;
    float weight_angle;
    float weight_first_cx_diff;
    float diff_dead_zone;
} Turn_info;

typedef struct {
    float dynamic_mid_line;
    float angle_err;
    float Yaw_err;
    float dynamic_zero_roll;
    float dynamic_zero_pitch;
    float yaw_increment;
    float dynamic_zero_threshold;
    float track_threshold;
    float turn_threshold;
    float dynamic_mid_line_kp;
    float dynamic_mid_line_offset;
    float dynamic_mid_line_kp_2;
    float dynamic_mid_line_offset_2;
    float img_mid_line;
    float angle_err_max;
} Mono_beacon;

typedef struct {
    float max_std;      //大目标方差下限
    float max_gray;  //大目标最大灰度下限
    float iou;             //交并比要求
    float min_std;          //被激活要求
    float min_gray;
    float max_miss;         //最大丢帧
    float max_distance;    //最远距离
    float img_process_mode;
    float small_area;
    float range_mean;
    float range_gray;
    float max_dx;
    float max_dy;
} Img_process_params;


#endif
