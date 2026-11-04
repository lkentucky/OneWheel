#include "HW_Drivers/IMU/imu.h"
#include "Common/board_config.h"
#include "Common/param.h"
#include <math.h>
#if defined(__TASKING__)
#pragma section all "cpu0_dsram"
#endif

imu_data_t imu;                         // 车体系加速度(g)与角速度(°/s)

static int s_forward_dir = IMU_FORWARD_DIR_BASE * IMU_FORWARD_DIR_DEFAULT; // 本次启动实际使用的坐标

static float gyro_bias_x, gyro_bias_y, gyro_bias_z;     // 静止标定得到的陀螺零偏

static float yaw_angle;                 // 转向外环用的连续航向角，永不清零
static float element_angle;             // 元素状态机使用的独立连续转角

static float static_acc_x = 0.0f;       // 标定期间的重力方向，供姿态冷启动对齐
static float static_acc_y = 0.0f;
static float static_acc_z = 1.0f;
static imu_calib_state_t calib_state = IMU_CALIB_NONE;  // 静止标定结果

static uint8  link_lost;                // 链路失效标志，一旦置位不再自动恢复

#define IMU_SETTLE_MS           (150)   // 改完 ODR 后等数字滤波器稳定的时间(ms)

#define IMU_CALIB_SAMPLES       (6000)   // 单次静止标定采样点数
#define IMU_CALIB_RETRY         (3)     // 检测到晃动后的重试次数
#define IMU_CALIB_MOVE_DPS      (3.0f)  // 标定期间陀螺峰峰值超过它判定为动过
#define IMU_CALIB_MOVE_G        (0.15f) // 标定期间加计峰峰值超过它判定为动过

static const char *s_imu_fault = "IMU LINK LOST";
static void imu_latch_fault(const char *reason)
{
    if (!link_lost) s_imu_fault = reason;
    link_lost = 1;
}
const char *imu_fault_text(void) { return s_imu_fault; }

#include "imu_spi.inc"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     把传感器轴换到车体轴，加速度与陀螺必须用同一套交换和同一套符号
// 参数说明     sx/sy/sz 传感器轴输入；bx/by/bz 车体轴输出
// 返回参数     void
// 使用示例     imu_sensor_to_body(sx, sy, sz, &bx, &by, &bz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_sensor_to_body(float sx, float sy, float sz,
                               float *bx, float *by, float *bz)
{
    *bx = (float)s_forward_dir * sy;       // 车体 X：左右倾，对应 Roll
    *by = (float)s_forward_dir * sz;       // 车体 Y：前后倾，对应 Pitch
    *bz = sx;       // 车体 Z：航向，对应 Yaw
}

#if (IMU660RB_USE_SOFT_IIC == 0)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     写入 IMU660RB 寄存器
// 参数说明     reg 寄存器地址；data 写入数据
// 返回参数     void
// 使用示例     imu_write_reg(IMU660RB_CTRL1_XL, value);
//-------------------------------------------------------------------------------------------------------------------
static void imu_write_reg(uint8 reg, uint8 data)
{
    IMU660RB_CS(0);
    spi_write_8bit_register(IMU660RB_SPI, (uint8)(reg | IMU660RB_SPI_W), data);
    IMU660RB_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU660RB 寄存器
// 参数说明     reg 寄存器地址
// 返回参数     uint8           寄存器数据
// 使用示例     uint8 value = imu_read_reg(IMU660RB_CTRL1_XL);
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_read_reg(uint8 reg)
{
    uint8 data=0xFFu;
    if (!r10h_bus_read(reg,&data,1u)) imu_latch_fault("IMU SPI READ FAIL");
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     配置 IMU660RB 采样率和数字滤波器
// 参数说明     void
// 返回参数     uint8           0 表示配置成功，1 表示回读失败
// 使用示例     uint8 state = imu_tune_sensor();
//-------------------------------------------------------------------------------------------------------------------
static uint8 imu_tune_sensor(void)
{
    /* Keep the full vendor initialization (BDU, auto-increment, filters).
     * Match the accepted reference: both ODRs 1667 Hz, +/-8g, +/-2000 dps. */
    imu_write_reg(IMU660RB_CTRL1_XL, 0x8Cu);
    imu_write_reg(IMU660RB_CTRL2_G, 0x8Cu);
    imu660rb_transition_factor[0] = 4098.0f;
    imu660rb_transition_factor[1] = 14.3f;
    system_delay_ms(5);
    return (uint8)(imu_read_reg(IMU660RB_WHO_AM_I) != 0x6Bu ||
        imu_read_reg(IMU660RB_CTRL1_XL) != 0x8Cu ||
        imu_read_reg(IMU660RB_CTRL2_G) != 0x8Cu ||
        (imu_read_reg(IMU660RB_CTRL3_C) & 0x44u) != 0x44u);
}

#else
#error "IMU660RB 必须用硬件 SPI: 软件 IIC 下库不暴露 IIC 句柄, ODR 只能停在 52/208Hz, 1kHz 控制环会一直读到重复样本"
#endif

#include "imu_r10h.inc"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴角速度
// 参数说明     bx 角速度 X 输出；by 角速度 Y 输出；bz 角速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_gyro_body(&gx, &gy, &gz);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_gyro_body(float *bx, float *by, float *bz)
{
    (void)r10h_read_body(); /* One complete six-axis burst per calibration/control sample. */
    *bx=r10h_raw_body.gyro_x; *by=r10h_raw_body.gyro_y; *bz=r10h_raw_body.gyro_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取车体系三轴加速度
// 参数说明     bx 加速度 X 输出；by 加速度 Y 输出；bz 加速度 Z 输出
// 返回参数     void
// 使用示例     imu_read_acc_body(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
static void imu_read_acc_body(float *bx, float *by, float *bz)
{
    /* Same burst as gyro, no second bus transaction / mixed sample. */
    *bx=r10h_raw_body.acc_x; *by=r10h_raw_body.acc_y; *bz=r10h_raw_body.acc_z;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化 IMU660RB 并等待滤波器稳定
// 参数说明     void
// 返回参数     uint8           0 表示成功，非 0 表示失败
// 使用示例     uint8 state = imu_init();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_init(void)
{
    uint8 state;

    s_forward_dir = (IMU_FORWARD_DIR < 0) ? -1 : 1;
    link_lost     = 0;
    s_imu_fault = "IMU LINK LOST";
    r10h_sample_valid = r10h_identity_ticks = 0;
    calib_state = IMU_CALIB_NONE;
    yaw_angle     = 0.0f;
    element_angle = 0.0f;

#ifndef R10H_HOST_TEST
    r10h_bus_timeout_ticks=(uint32)(IfxStm_getFrequency(&MODULE_STM0)*0.0001f);
#endif
    state = imu660rb_init();          
    if (state != 0)
        return state;

    if (imu_tune_sensor() != 0)
        return 1;

    
    system_delay_ms(IMU_SETTLE_MS);
    return 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     静止标定陀螺仪零偏和重力方向
// 参数说明     void
// 返回参数     void
// 使用示例     imu_calibrate();
//-------------------------------------------------------------------------------------------------------------------
void imu_calibrate(void)
{
    uint8 attempt;

    calib_state = IMU_CALIB_NONE;
    for (attempt = 0; attempt < IMU_CALIB_RETRY; ++attempt)
    {
        float g_sum[3] = {0.0f, 0.0f, 0.0f};
        float a_sum[3] = {0.0f, 0.0f, 0.0f};
        float g_min[3] = {1e30f, 1e30f, 1e30f}, g_max[3] = {-1e30f, -1e30f, -1e30f};
        float a_min[3] = {1e30f, 1e30f, 1e30f}, a_max[3] = {-1e30f, -1e30f, -1e30f};
        uint8 moved = 0;
        uint16 i;
        uint8 k;

        for (i = 0; i < IMU_CALIB_SAMPLES; ++i)
        {
            float g[3], a[3];

            imu_read_gyro_body(&g[0], &g[1], &g[2]);
            imu_read_acc_body(&a[0], &a[1], &a[2]);
            if (link_lost) return; // Invalid samples never enter the startup mean.

            for (k = 0; k < 3u; ++k)
            {
                g_sum[k] += g[k];
                a_sum[k] += a[k];
                if (g[k] < g_min[k]) g_min[k] = g[k];
                if (g[k] > g_max[k]) g_max[k] = g[k];
                if (a[k] < a_min[k]) a_min[k] = a[k];
                if (a[k] > a_max[k]) a_max[k] = a[k];
            }
            system_delay_ms(1);
        }

        {
            float inv = 1.0f / (float)IMU_CALIB_SAMPLES;
            gyro_bias_x = g_sum[0] * inv;
            gyro_bias_y = g_sum[1] * inv;
            gyro_bias_z = g_sum[2] * inv;
            static_acc_x = a_sum[0] * inv;
            static_acc_y = a_sum[1] * inv;
            static_acc_z = a_sum[2] * inv;
        }

        for (k = 0; k < 3u; ++k)
            if ((g_max[k] - g_min[k]) > IMU_CALIB_MOVE_DPS ||
                (a_max[k] - a_min[k]) > IMU_CALIB_MOVE_G)
                moved = 1;

        {
            float norm = sqrtf(static_acc_x*static_acc_x + static_acc_y*static_acc_y + static_acc_z*static_acc_z);
            if (norm < 0.75f || norm > 1.25f) moved = 1;
        }
        if (!moved)
        {
            calib_state = IMU_CALIB_OK;
            break;
        }
        calib_state = IMU_CALIB_MOVED;      
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取 IMU 标定状态
// 参数说明     void
// 返回参数     imu_calib_state_t 当前标定状态
// 使用示例     imu_calib_state_t state = imu_calib_state();
//-------------------------------------------------------------------------------------------------------------------
imu_calib_state_t imu_calib_state(void)
{
    return calib_state;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取标定期间采集的车体系重力方向
// 参数说明     ax 加速度 X 输出；ay 加速度 Y 输出；az 加速度 Z 输出
// 返回参数     uint8           1 表示数据有效，0 表示尚未标定
// 使用示例     uint8 valid = imu_get_static_acc(&ax, &ay, &az);
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_get_static_acc(float *ax, float *ay, float *az)
{
    *ax = static_acc_x;
    *ay = static_acc_y;
    *az = static_acc_z;
    return (uint8)(calib_state == IMU_CALIB_OK && !link_lost);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     每 1 ms 采集完整六轴，并更新车体系角速度
// 参数说明     void
// 返回参数     void
// 使用示例     imu_update_gyro();
//-------------------------------------------------------------------------------------------------------------------
void imu_update_gyro(void)
{
    float bx, by, bz;
    imu_read_gyro_body(&bx, &by, &bz);
    if (link_lost) return;
    imu_read_acc_body(&imu.acc_x, &imu.acc_y, &imu.acc_z);
    /* Controllers retain bias-corrected, unfiltered 1 ms body rates.
     * A+ gets the separate raw sample and removes startup bias internally. */
    imu.gyro_x=bx-gyro_bias_x; imu.gyro_y=by-gyro_bias_y; imu.gyro_z=bz-gyro_bias_z;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     检查 IMU660RB 通信链路状态
// 参数说明     void
// 返回参数     uint8           1 表示链路失效，0 表示正常
// 使用示例     uint8 lost = imu_link_lost();
//-------------------------------------------------------------------------------------------------------------------
uint8 imu_forward_pending(void)
{
    return (uint8)(s_forward_dir != IMU_FORWARD_DIR);
}

uint8 imu_link_lost(void)
{
    return link_lost;
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     发布姿态解算得到的航向角和本周期增量
// 参数说明     yaw_continuous 连续航向角；yaw_delta 本周期航向增量
// 返回参数     void
// 使用示例     imu_publish_attitude_yaw(att.yaw, yaw_delta);
//-------------------------------------------------------------------------------------------------------------------
void imu_publish_attitude_yaw(float yaw_continuous, float yaw_delta)
{
    yaw_angle = yaw_continuous;
    element_angle += yaw_delta;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取连续航向角
// 参数说明     void
// 返回参数     float           连续航向角，单位 deg
// 使用示例     float yaw = imu_get_angle_yaw();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_yaw(void)
{
    return yaw_angle;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取元素处理使用的独立累计转角
// 参数说明     void
// 返回参数     float           元素累计转角，单位 deg
// 使用示例     float angle = imu_get_angle_element();
//-------------------------------------------------------------------------------------------------------------------
float imu_get_angle_element(void)
{
    return element_angle;
}

#if defined(__TASKING__)
#pragma section all restore
#endif
