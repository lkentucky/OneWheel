/*
 * wifi_link.c
 *
 * WiFi SPI 无线链路实现（TC264 + 逐飞 WiFi SPI 模块）
 *
 * 移植说明：
 *   1. 沿用 MUDP/MCAM 封装；独轮原生调参表由 tools/monitor_onewheel.py 提供；
 *   2. RUN 使用 wifi_link_io 分步收发；原驱动只负责初始化和连接，
 *      因此这里把整个链路放进 CPU0 的空主循环里，用状态机 + 时间片的方式调度，
 *      CPU1 主循环与控制中断均不调用 WiFi SPI 收发；其它外设保持原有接口；
 *   3. 每次 SPI 事务前先看 INT 引脚（高=模块空闲），模块忙时直接让出，
 *      每次调用最多一次 CS 事务，忙时让出；250ms 超时后重连；
 *   4. 所有发送缓冲都放在默认段（dsram1，0x60000000），CPU0/CPU1 都能访问，
 *      与工程里 encoder_data_1、image_binary 等跨核变量的存放方式一致。
 */

#include "zf_common_headfile.h"
#include "wifi_link.h"
#include "wifi_link_io.h"
#include <errno.h>
#include <float.h>
#include <math.h>

#include <stdarg.h>
#include <stdlib.h>

#include "IfxStm.h"

//=====================================================协议常量=====================================================
#define WL_MUX_HEADER_SIZE      (8)
#define WL_DATA_SIZE            (148)
#define WL_CAM_HEADER_SIZE      (24)
#define WL_CAM_CHUNK_SIZE       (1400)
#define WL_CAM_CHUNK_COUNT      ((WIFI_LINK_IMG_SIZE + WL_CAM_CHUNK_SIZE - 1) / WL_CAM_CHUNK_SIZE)
#define WL_PACKET_SIZE          (WL_MUX_HEADER_SIZE + WL_CAM_HEADER_SIZE + WL_CAM_CHUNK_SIZE)
#define WL_LOG_CHUNK_SIZE       (900)
#define WL_CMD_SIZE             (64)

#define WL_MUX_TYPE_DATA        (1)
#define WL_MUX_TYPE_LOG         (2)
#define WL_MUX_TYPE_CAMERA      (3)

//=====================================================共享数据=====================================================
// CPU1 写、CPU0 读；序列号和 DSYNC 避免读取到混合遥测快照
typedef struct
{
    volatile uint32 sequence;
    volatile uint32 cpu1_ready;
    float  roll_deg;
    float  pitch_deg;
    float  yaw_deg;
    float  gx, gy, gz;
    float  ax, ay, az;
    float  erro_px;
    float  omega_cmd;
    float  vx_cmd;
    uint32 vision_interval_us;
    uint32 vision_frames;
    uint32 vision_event;
    volatile uint32 frame_want;         // CPU0 -> CPU1：请求一帧图像
    volatile uint32 frame_ready;        // CPU1 -> CPU0：图像已拷贝完成
    uint32 frame_index;                 // 当前帧号（CPU1 递增）
} wl_share_struct;

static volatile wl_share_struct wl_share;

volatile float wifi_target_speed = 160.0f;       // 行进轮目标速度（原控制环里的固定 160）
volatile float wifi_ramp_step = 0.03f;
volatile float wifi_ramp_ms = 160.0f / 0.03f;
volatile float wifi_deadband_px = 0.0f;
volatile uint32 wifi_encoder_total = 0;           // 行进轮编码器累计计数（控制中断里累加）

//=====================================================缓冲区（dsram1）=====================================================
static uint8 wl_frame[WIFI_LINK_IMG_SIZE];                          // 22560B 图像快照
static uint8 wl_packet[WL_PACKET_SIZE];                             // 8+24+1400
static char  wl_cmd[WL_CMD_SIZE];
static uint8 wl_cmd_len;
static uint8 wl_cmd_discard;
static char  wl_c1_log[1024];           // CPU1 -> CPU0 日志环形缓冲
static volatile uint16 wl_c1_head;
static volatile uint16 wl_c1_tail;
static char  wl_c0_log[2048];           // CPU0 自己的日志/参数应答
static uint16 wl_c0_len;

static char wl_host[16] = WIFI_LINK_HOST_IP;
static char wl_next_host[16];

//=====================================================状态机=====================================================
typedef enum
{
    WL_ST_BOOT = 0,         // 上电等待
    WL_ST_INIT,             // 模块初始化 + 连 WiFi
    WL_ST_SOCKET,           // 建立 UDP socket
    WL_ST_RUN,              // 正常收发
    WL_ST_BACKOFF,          // 失败退避
} wl_state_enum;

static wl_state_enum wl_state = WL_ST_BOOT;
static wl_state_enum wl_backoff_next = WL_ST_INIT;
static uint32 wl_backoff_ms;

static uint32 wl_seq;
static uint8  wl_tx_fail;
static uint8  wl_host_seen;
static uint8  wl_socket_fail;

// 时间基准：直接读 STM0，避免依赖 PIT（PIT 由 CPU1 初始化，上电早期还没跑）
static uint32 wl_tick_last;
static uint32 wl_tick_frac;
static uint32 wl_tick_per_ms;
static uint32 wl_tick_per_us;
static uint32 wl_now_ms;

static uint32 wl_last_data_ms;
static uint32 wl_last_log_ms;
static uint32 wl_last_rx_ms;
static uint32 wl_last_image_ms;

static uint32 wl_frame_pos;             // 当前帧已发分片数
static uint32 wl_frame_crc;
static uint32 wl_frame_done_ms;
static uint8 wl_frame_crc_valid;
static uint32 wl_idle_busy_ms;
typedef enum { WL_TX_NONE, WL_TX_DATA, WL_TX_LOG0, WL_TX_LOG1, WL_TX_IMAGE } wl_tx_kind;
static wl_tx_kind wl_pending_kind;
static uint16 wl_pending_len;
static void wl_command_bytes(const uint8 *bytes, uint32 length);
static void wl_backoff(wl_state_enum next, const char *reason);

//=====================================================小工具=====================================================
static void wl_put_u16(uint8 *p, uint16 v)
{
    p[0] = (uint8)(v & 0xFF);
    p[1] = (uint8)(v >> 8);
}

static void wl_put_u32(uint8 *p, uint32 v)
{
    p[0] = (uint8)(v & 0xFF);
    p[1] = (uint8)((v >> 8) & 0xFF);
    p[2] = (uint8)((v >> 16) & 0xFF);
    p[3] = (uint8)((v >> 24) & 0xFF);
}

static void wl_put_float(uint8 *p, float v)
{
    uint32 u;
    memcpy(&u, &v, sizeof(u));
    wl_put_u32(p, u);
}

static void wl_mux_header(uint8 *p, uint8 type, uint16 length)
{
    memcpy(p, "MUDP", 4);
    p[4] = 1;
    p[5] = type;
    wl_put_u16(p + 6, length);
}

static uint32 wl_crc32(const uint8 *data, uint32 length)
{
    static const uint32 table[16] =
    {
        0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu,
        0x76DC4190u, 0x6B6B51F4u, 0x4DB26158u, 0x5005713Cu,
        0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
        0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
    };
    uint32 crc = 0xFFFFFFFFu;
    while (length--)
    {
        crc ^= *data++;
        crc = (crc >> 4) ^ table[crc & 0x0F];
        crc = (crc >> 4) ^ table[crc & 0x0F];
    }
    return crc ^ 0xFFFFFFFFu;
}

static void wl_c0_append(const char *text)
{
    while (*text && wl_c0_len < sizeof(wl_c0_log))
    {
        wl_c0_log[wl_c0_len++] = *text++;
    }
}

void wifi_link_log(const char *format, ...)
{
    char text[128];
    va_list args;
    va_start(args, format);
    vsnprintf(text, sizeof(text), format, args);
    va_end(args);
    wl_c0_append(text);
}

void wifi_link_cpu1_log(const char *text)
{
    uint16 next;
    if (NULL == text)
    {
        return;
    }
    while (*text)
    {
        next = (uint16)((wl_c1_head + 1) % sizeof(wl_c1_log));
        if (next == wl_c1_tail)
        {
            return;                     // 缓冲满，丢弃
        }
        wl_c1_log[wl_c1_head] = *text++;
        __dsync();
        wl_c1_head = next;
    }
    next = (uint16)((wl_c1_head + 1) % sizeof(wl_c1_log));
    if (next != wl_c1_tail)
    {
        wl_c1_log[wl_c1_head] = '\n';
        __dsync();
        wl_c1_head = next;
    }
}

uint8 wifi_link_online(void)
{
    return (WL_ST_RUN == wl_state) ? 1 : 0;
}

//=====================================================时间与调度=====================================================
static void wl_time_init(void)
{
    float freq = IfxStm_getFrequency(&MODULE_STM0);
    wl_tick_per_ms = (uint32)(freq / 1000.0f);
    if (!wl_tick_per_ms) { wl_tick_per_ms = 1u; }
    wl_tick_per_us = (wl_tick_per_ms >= 1000u) ? (wl_tick_per_ms / 1000u) : 1u;
    wl_tick_last = IfxStm_getLower(&MODULE_STM0);
    wl_tick_frac = 0;
    wl_now_ms = 0;
}

static void wl_time_poll(void)
{
    uint32 now = IfxStm_getLower(&MODULE_STM0);
    uint32 delta = now - wl_tick_last;
    wl_tick_last = now;
    wl_now_ms += delta / wl_tick_per_ms;
    wl_tick_frac += delta % wl_tick_per_ms;
    if (wl_tick_frac >= wl_tick_per_ms)
    {
        wl_tick_frac -= wl_tick_per_ms;
        wl_now_ms++;
    }
}

// 模块 INT 为高表示空闲；为低表示模块正在处理上一包
static uint8 wl_module_idle(void)
{
    return gpio_get_level(WIFI_SPI_INT_PIN) ? 1 : 0;
}

//=====================================================发送=====================================================
static uint8 wl_send_wire(uint8 *buffer, uint16 length)
{
    return wifi_link_io_send(buffer, length, wl_now_ms);
}

static void wl_tx_committed(void)
{
    switch (wl_pending_kind)
    {
        case WL_TX_LOG0:
            if (wl_pending_len <= wl_c0_len)
            {
                wl_c0_len = (uint16)(wl_c0_len - wl_pending_len);
                memmove(wl_c0_log, wl_c0_log + wl_pending_len, wl_c0_len);
            }
            break;
        case WL_TX_LOG1:
            __dsync();
            wl_c1_tail = (uint16)((wl_c1_tail + wl_pending_len) % sizeof(wl_c1_log));
            break;
        case WL_TX_IMAGE:
            wl_frame_pos++;
            if (wl_frame_pos >= WL_CAM_CHUNK_COUNT)
            {
                wl_frame_pos = 0;
                wl_frame_crc_valid = 0;
                __dsync();
                wl_share.frame_ready = 0;
                wl_frame_done_ms = wl_now_ms;
                wl_last_image_ms = wl_now_ms;
            }
            break;
        default: break;
    }
    wl_pending_kind = WL_TX_NONE;
}

static void wl_io_service(void)
{
    wifi_link_io_result result = wifi_link_io_poll(wl_now_ms);
    if (result == WIFI_LINK_IO_SENT)
    {
        wl_tx_committed();
    }
    else if (result == WIFI_LINK_IO_RECEIVED)
    {
        /* Measure from completion so slow RX cannot starve outbound work. */
        wl_last_rx_ms = wl_now_ms;
    }
    else if (result == WIFI_LINK_IO_ERROR)
    {
        /* Keep queued log bytes and restart any partial camera frame. */
        wl_pending_kind = WL_TX_NONE;
        wl_frame_pos = 0;
        wl_cmd_len = 0;
        wl_cmd_discard = 0;
        wl_backoff(WL_ST_INIT, "runtime SPI timeout/error");
    }
}

static void wl_send_data(void)
{
    uint8 *p = wl_packet + WL_MUX_HEADER_SIZE;
    uint32 sequence = wl_share.sequence;
    float roll, pitch, yaw;

    if (!wl_module_idle())
    {
        return;                                             // 模块忙时直接跳过，下个 tick 再试
    }
    if (!wl_share.cpu1_ready || (sequence & 1u)) { return; }
    __dsync();
    roll = wl_share.roll_deg; pitch = wl_share.pitch_deg; yaw = wl_share.yaw_deg;
    wl_mux_header(wl_packet, WL_MUX_TYPE_DATA, WL_DATA_SIZE);
    memset(p, 0, WL_DATA_SIZE);

    wl_put_float(p + 0,  roll);
    wl_put_float(p + 4,  pitch);
    wl_put_float(p + 8,  yaw);

    wl_put_float(p + 12, wl_share.gx);
    wl_put_float(p + 16, wl_share.gy);
    wl_put_float(p + 20, wl_share.gz);

    wl_put_float(p + 24, wl_share.ax);
    wl_put_float(p + 28, wl_share.ay);
    wl_put_float(p + 32, wl_share.az);

    wl_put_u32(p + 36, (uint32)wifi_encoder_total);         // FL：行进轮累计编码器
    wl_put_u32(p + 40, 0);                                  // FR
    wl_put_u32(p + 44, 0);                                  // RL
    wl_put_u32(p + 48, 0);                                  // RR

    wl_put_u32(p + 52, system_count);                       // 采样计数（1ms）
    wl_put_u32(p + 56, wl_seq++);

    wl_put_float(p + 60, expect_speed);                     // target[0] 行进轮目标速度
    wl_put_float(p + 64, 0.0f);
    wl_put_float(p + 68, 0.0f);
    wl_put_float(p + 72, 0.0f);

    wl_put_float(p + 76, (float)encoder_data_1);            // speed[0] 行进轮实际
    wl_put_float(p + 80, (float)motor_value.receive_left_speed_data);
    wl_put_float(p + 84, (float)motor_value.receive_right_speed_data);
    wl_put_float(p + 88, (float)car_speed);

    wl_put_float(p + 92, (float)bujin_last_duty);           // duty[0] 行进轮输出
    wl_put_float(p + 96, 0.0f);
    wl_put_float(p + 100, 0.0f);
    wl_put_float(p + 104, 0.0f);

    wl_put_float(p + 108, wl_share.erro_px);
    wl_put_float(p + 112, wl_share.omega_cmd);
    wl_put_float(p + 116, wl_share.vx_cmd);
    wl_put_float(p + 120, 0.0f);

    wl_put_u32(p + 124, 0);                                 // vis_time_us（暂不测量）
    wl_put_u32(p + 128, wl_share.vision_interval_us);
    wl_put_u32(p + 132, wl_share.vision_frames);
    wl_put_u32(p + 136, wifi_link_io_last_us());                                 // wifi_time_us
    wl_put_u32(p + 140, wifi_link_io_peak_us());                                 // wifi_peak_us
    wl_put_u32(p + 144, wl_share.vision_event);

    __dsync();
    if (sequence != wl_share.sequence) { return; }
    if (0 == wl_send_wire(wl_packet, (uint16)(WL_MUX_HEADER_SIZE + WL_DATA_SIZE)))
    {
        wl_pending_kind = WL_TX_DATA;
    }
    wl_last_data_ms = wl_now_ms;
}

static void wl_send_logs(void)
{
    uint16 n = 0;
    uint16 tail = wl_c1_tail;
    if (!wl_module_idle()) { return; }
    wl_last_log_ms = wl_now_ms;
    /* Parameter acknowledgements take priority over streaming debug text. */
    if (wl_c0_len)
    {
        n = wl_c0_len > WL_LOG_CHUNK_SIZE ? WL_LOG_CHUNK_SIZE : wl_c0_len;
        memcpy(wl_packet + WL_MUX_HEADER_SIZE, wl_c0_log, n);
        wl_pending_kind = WL_TX_LOG0;
    }
    else
    {
        __dsync();
        while (tail != wl_c1_head && n < WL_LOG_CHUNK_SIZE)
        {
            wl_packet[WL_MUX_HEADER_SIZE + n++] = (uint8)wl_c1_log[tail];
            tail = (uint16)((tail + 1) % sizeof(wl_c1_log));
        }
        wl_pending_kind = WL_TX_LOG1;
    }
    if (n)
    {
        wl_mux_header(wl_packet, WL_MUX_TYPE_LOG, n);
        wl_pending_len = n;
        if (wl_send_wire(wl_packet, (uint16)(WL_MUX_HEADER_SIZE + n)))
        {
            wl_pending_kind = WL_TX_NONE;
        }
    }
    else { wl_pending_kind = WL_TX_NONE; }
}

// 每调用一次发一片；发完 17 片才算一帧
static void wl_send_image_slice(void)
{
    uint32 offset;
    uint16 length;
    uint16 count = (uint16)WL_CAM_CHUNK_COUNT;

    if (!wl_share.frame_ready)
    {
        // 请求下一帧
        if (wl_share.frame_want)
        {
            if ((wl_now_ms - wl_frame_done_ms) > 300u)       // CPU1 长时间不响应则撤回请求
            {
                wl_share.frame_want = 0;
            }
        }
        else if ((wl_now_ms - wl_last_image_ms) >= WIFI_LINK_IMAGE_PERIOD_MS)
        {
            wl_share.frame_want = 1;
            wl_last_image_ms = wl_now_ms;
            wl_frame_done_ms = wl_now_ms;
        }
        return;
    }

    if (!wl_frame_crc_valid)
    {
        __dsync();
        wl_frame_crc = wl_crc32(wl_frame, WIFI_LINK_IMG_SIZE);
        wl_frame_crc_valid = 1;
    }

    if (!wl_module_idle())
    {
        return;                                             // 模块忙，下个时间片再来
    }

    offset = wl_frame_pos * WL_CAM_CHUNK_SIZE;
    length = (uint16)((WIFI_LINK_IMG_SIZE - offset) > WL_CAM_CHUNK_SIZE ?
                      WL_CAM_CHUNK_SIZE : (WIFI_LINK_IMG_SIZE - offset));

    memcpy(wl_packet + WL_MUX_HEADER_SIZE, "MCAM", 4);
    wl_packet[WL_MUX_HEADER_SIZE + 4] = 1;                  // 版本
    wl_packet[WL_MUX_HEADER_SIZE + 5] = 1;                  // 灰度 8bit
    wl_put_u16(wl_packet + WL_MUX_HEADER_SIZE + 6,  WIFI_LINK_IMG_W);
    wl_put_u16(wl_packet + WL_MUX_HEADER_SIZE + 8,  WIFI_LINK_IMG_H);
    wl_put_u32(wl_packet + WL_MUX_HEADER_SIZE + 10, wl_share.frame_index);
    wl_put_u16(wl_packet + WL_MUX_HEADER_SIZE + 14, (uint16)wl_frame_pos);
    wl_put_u16(wl_packet + WL_MUX_HEADER_SIZE + 16, count);
    wl_put_u16(wl_packet + WL_MUX_HEADER_SIZE + 18, length);
    wl_put_u32(wl_packet + WL_MUX_HEADER_SIZE + 20, wl_frame_crc);

    memcpy(wl_packet + WL_MUX_HEADER_SIZE + WL_CAM_HEADER_SIZE, wl_frame + offset, length);
    wl_mux_header(wl_packet, WL_MUX_TYPE_CAMERA, (uint16)(WL_CAM_HEADER_SIZE + length));
    if (0 == wl_send_wire(wl_packet, (uint16)(WL_MUX_HEADER_SIZE + WL_CAM_HEADER_SIZE + length)))
    {
        wl_pending_kind = WL_TX_IMAGE;
    }
}

float wifi_vision_erro(float erro_value)
{
    if (wifi_deadband_px > 0.0f)
    {
        if ((erro_value < wifi_deadband_px) && (erro_value > -wifi_deadband_px))
        {
            return 0.0f;
        }
    }
    return erro_value;
}

//=====================================================参数表=====================================================
typedef struct
{
    const char *name;
    volatile float *value;
    float       min;
    float       max;
} wl_param_struct;

static const wl_param_struct wl_params[] =
{
    {"speed",         &wifi_target_speed,                       -1000.0f, 1000.0f},
    {"k1",            &k1,                                        -10.0f,   10.0f},
    {"k2",            &k2,                                        -10.0f,   10.0f},

    {"roll_angle_p",  &balance_cascade.angle_cycle.p,           -10000.0f, 10000.0f},
    {"roll_angle_i",  &balance_cascade.angle_cycle.i,           -10000.0f, 10000.0f},
    {"roll_angle_d",  &balance_cascade.angle_cycle.d,           -10000.0f, 10000.0f},
    {"roll_speed_p",  &balance_cascade.speed_cycle.p,           -10000.0f, 10000.0f},
    {"roll_speed_i",  &balance_cascade.speed_cycle.i,           -10000.0f, 10000.0f},
    {"roll_speed_d",  &balance_cascade.speed_cycle.d,           -10000.0f, 10000.0f},
    {"roll_rate_p",   &balance_cascade.angular_speed_cycle.p,   -10000.0f, 10000.0f},
    {"roll_rate_i",   &balance_cascade.angular_speed_cycle.i,   -10000.0f, 10000.0f},
    {"roll_rate_d",   &balance_cascade.angular_speed_cycle.d,   -10000.0f, 10000.0f},

    {"pitch_angle_p", &balance_cascade.pitch_angle_cycle.p,     -10000.0f, 10000.0f},
    {"pitch_angle_i", &balance_cascade.pitch_angle_cycle.i,     -10000.0f, 10000.0f},
    {"pitch_angle_d", &balance_cascade.pitch_angle_cycle.d,     -10000.0f, 10000.0f},
    {"pitch_speed_p", &balance_cascade.pitch_speed_cycle.p,     -10000.0f, 10000.0f},
    {"pitch_speed_i", &balance_cascade.pitch_speed_cycle.i,     -10000.0f, 10000.0f},
    {"pitch_speed_d", &balance_cascade.pitch_speed_cycle.d,     -10000.0f, 10000.0f},
    {"pitch_rate_p",  &balance_cascade.pitch_angular_speed_cycle.p, -10000.0f, 10000.0f},
    {"pitch_rate_i",  &balance_cascade.pitch_angular_speed_cycle.i, -10000.0f, 10000.0f},
    {"pitch_rate_d",  &balance_cascade.pitch_angular_speed_cycle.d, -10000.0f, 10000.0f},

    {"motor_p",       &balance_cascade.motor_cycle.p,           -10000.0f, 10000.0f},
    {"motor_i",       &balance_cascade.motor_cycle.i,           -10000.0f, 10000.0f},
    {"motor_d",       &balance_cascade.motor_cycle.d,           -10000.0f, 10000.0f},

    {"yaw_p",         &balance_cascade.yaw_erro_cycle.p,        -10000.0f, 10000.0f},
    {"yaw_i",         &balance_cascade.yaw_erro_cycle.i,        -10000.0f, 10000.0f},
    {"yaw_d",         &balance_cascade.yaw_erro_cycle.d,        -10000.0f, 10000.0f},
    {"yaw_rate_p",    &balance_cascade.yaw_angular_speed_cycle.p, -10000.0f, 10000.0f},
    {"yaw_rate_i",    &balance_cascade.yaw_angular_speed_cycle.i, -10000.0f, 10000.0f},
    {"yaw_rate_d",    &balance_cascade.yaw_angular_speed_cycle.d, -10000.0f, 10000.0f},
};

#define WL_PARAM_COUNT  (sizeof(wl_params) / sizeof(wl_params[0]))

static const wl_param_struct *wl_param_find(const char *name)
{
    uint16 i;
    for (i = 0; i < WL_PARAM_COUNT; i++)
    {
        if (0 == strcmp(wl_params[i].name, name))
        {
            return &wl_params[i];
        }
    }
    return NULL;
}

//=====================================================命令解析=====================================================
static uint8 wl_split(char *line, char *token[], uint8 max)
{
    uint8 count = 0;
    char *p = line;

    while (*p && count < max)
    {
        while ((' ' == *p) || ('\t' == *p))
        {
            p++;
        }
        if (!*p)
        {
            break;
        }
        token[count++] = p;
        while (*p && (' ' != *p) && ('\t' != *p))
        {
            p++;
        }
        if (*p)
        {
            *p++ = '\0';
        }
    }
    return count;
}

static void wl_reply_ok(const char *id, const char *name, float value)
{
    if (WL_ST_RUN != wl_state)
    {
        return;
    }
    wifi_link_log("[PARAM] %s OK %s %.9g\r\n", id, name, (double)value);
}

static void wl_reply_done(const char *id)
{
    if (WL_ST_RUN != wl_state)
    {
        return;
    }
    wifi_link_log("[PARAM] %s DONE\r\n", id);
}

static void wl_reply_err(const char *id, const char *reason)
{
    if (WL_ST_RUN != wl_state)
    {
        return;
    }
    wifi_link_log("[PARAM] %s ERR %s\r\n", id, reason);
}

static uint8 wl_number(const char *text, float *value)
{
    char *end;
    double number;
    errno = 0;
    number = strtod(text, &end);
    if (end == text || *end || errno == ERANGE ||
        !(number >= -FLT_MAX && number <= FLT_MAX)) { return 0; }
    *value = (float)number;
    return 1;
}

static uint8 wl_ipv4(const char *text)
{
    unsigned int a, b, c, d;
    char extra;
    if (strlen(text) >= sizeof(wl_next_host) ||
        sscanf(text, "%u.%u.%u.%u%c", &a, &b, &c, &d, &extra) != 4) { return 0; }
    return a > 0 && a < 224 && b <= 255 && c <= 255 && d <= 255;
}

static void wl_command_line(char *line)
{
    char *token[6];
    char *end;
    uint8 count = wl_split(line, token, 6);
    unsigned long id;
    uint16 i;
    float value;
    const wl_param_struct *param;
    if (count < 3 || strcmp(token[0], "MCAR")) { return; }
    errno = 0;
    id = strtoul(token[2], &end, 10);
    if (end == token[2] || *end || errno == ERANGE || id > 2147483647UL) { return; }
    /* Do not apply a write when its acknowledgement cannot fit in the queue. */
    if (wl_c0_len > sizeof(wl_c0_log) - 1800u) { return; }
    if (!strcmp(token[1], "ALIVE") && count == 3)
    {
        wl_host_seen = 1;
        return;
    }
    if (!strcmp(token[1], "GET") && count == 4 && !strcmp(token[3], "ALL"))
    {
        if (!wl_share.cpu1_ready) { wl_reply_err(token[2], "NOT_READY"); return; }
        for (i = 0; i < WL_PARAM_COUNT; i++)
        {
            wl_reply_ok(token[2], wl_params[i].name, *wl_params[i].value);
        }
        wl_reply_ok(token[2], "run", (float)run_flag);
        wl_reply_ok(token[2], "ramp_ms", wifi_ramp_ms);
        wl_reply_ok(token[2], "deadband_px", wifi_deadband_px);
        if (WIFI_LINK_CM_PER_COUNT > 0.0f)
        {
            wl_reply_ok(token[2], "cruise_cms", wifi_target_speed * 50.0f * WIFI_LINK_CM_PER_COUNT);
        }
        wl_reply_done(token[2]);
        return;
    }
    if (strcmp(token[1], "SET") || count != 5) { wl_reply_err(token[2], "SYNTAX"); return; }
    if (!strcmp(token[3], "host"))
    {
        if (!wl_ipv4(token[4])) { wl_reply_err(token[2], "RANGE"); return; }
        strcpy(wl_next_host, token[4]);
        wl_reply_ok(token[2], "host", 0.0f);
        return; /* Reconnect in SOCKET only after the old endpoint receives ACK. */
    }
    if (!wl_number(token[4], &value)) { wl_reply_err(token[2], "NUMBER"); return; }
    if (!strcmp(token[3], "run"))
    {
        if (value != 0.0f) { wl_reply_err(token[2], "LOCAL_START_REQUIRED"); return; }
        run_flag = 0;
        wl_reply_ok(token[2], "run", 0.0f);
        return;
    }
    if (!wl_share.cpu1_ready) { wl_reply_err(token[2], "NOT_READY"); return; }
    if (!strcmp(token[3], "cruise_cms"))
    {
        float calibration = WIFI_LINK_CM_PER_COUNT;
        if (calibration <= 0.0f) { wl_reply_err(token[2], "UNCALIBRATED"); return; }
        if (value < 0.0f || value > 200.0f || value / (50.0f * calibration) > 1000.0f)
        { wl_reply_err(token[2], "RANGE"); return; }
        wifi_target_speed = value / (50.0f * calibration);
        wl_reply_ok(token[2], "cruise_cms", value);
        return;
    }
    if (!strcmp(token[3], "ramp_ms"))
    {
        if (value < 100.0f || value > 20000.0f) { wl_reply_err(token[2], "RANGE"); return; }
        if (run_flag) { wl_reply_err(token[2], "STOP_FIRST"); return; }
        wifi_ramp_ms = value;
        wifi_ramp_step = 160.0f / value;
        wl_reply_ok(token[2], "ramp_ms", value);
        return;
    }
    if (!strcmp(token[3], "deadband_px"))
    {
        if (value < 0.0f || value > 10.0f) { wl_reply_err(token[2], "RANGE"); return; }
        wifi_deadband_px = value;
        wl_reply_ok(token[2], "deadband_px", value);
        return;
    }
    param = wl_param_find(token[3]);
    if (!param) { wl_reply_err(token[2], "UNKNOWN_KEY"); return; }
    if (value < param->min || value > param->max) { wl_reply_err(token[2], "RANGE"); return; }
    if (param != &wl_params[0] && run_flag) { wl_reply_err(token[2], "STOP_FIRST"); return; }
    *param->value = value;
    wl_reply_ok(token[2], param->name, value);
}

static void wl_command_bytes(const uint8 *bytes, uint32 length)
{
    uint32 i;
    uint8 c;
    for (i = 0; i < length; i++)
    {
        c = bytes[i];
        if (c == '\n')
        {
            if (!wl_cmd_discard && wl_cmd_len)
            {
                wl_cmd[wl_cmd_len] = '\0';
                wl_command_line(wl_cmd);
            }
            wl_cmd_len = 0;
            wl_cmd_discard = 0;
        }
        else if (c == '\r') { continue; }
        else if (wl_cmd_discard) { continue; }
        else if ((c < 32 && c != '\t') || c > 126 || wl_cmd_len >= WL_CMD_SIZE - 1)
        {
            wl_cmd_discard = 1;
            wl_cmd_len = 0;
        }
        else { wl_cmd[wl_cmd_len++] = (char)c; }
    }
}

static void wl_enter(wl_state_enum state)
{
    wl_state = state;
}

static void wl_backoff(wl_state_enum next, const char *reason)
{
    wifi_link_log("[NET] %s, retry in %dms\r\n", reason, WIFI_LINK_RETRY_MS);
    wl_backoff_next = next;
    wl_backoff_ms = wl_now_ms;
    wl_enter(WL_ST_BACKOFF);
}

static void wl_run(void)
{
    if (wifi_link_io_busy()) { wl_io_service(); return; }
    if (wl_next_host[0] && !wl_c0_len)
    {
        strcpy(wl_host, wl_next_host);
        wl_next_host[0] = '\0';
        wl_state = WL_ST_SOCKET;
        return;
    }
    if (!wl_module_idle())
    {
        if (!wl_idle_busy_ms) { wl_idle_busy_ms = wl_now_ms; }
        if ((uint32)(wl_now_ms - wl_idle_busy_ms) >= 250u)
        {
            wl_backoff(WL_ST_INIT, "module busy timeout");
        }
        return;
    }
    wl_idle_busy_ms = 0;
    if ((uint32)(wl_now_ms - wl_last_rx_ms) >= 5u)
    {
        wifi_link_io_receive(wl_now_ms);
        wl_last_rx_ms = wl_now_ms;
    }
    else if ((wl_c0_len || wl_c1_head != wl_c1_tail) &&
             (uint32)(wl_now_ms - wl_last_log_ms) >= WIFI_LINK_LOG_PERIOD_MS)
    {
        wl_send_logs();
    }
    else if ((uint32)(wl_now_ms - wl_last_data_ms) >= WIFI_LINK_DATA_PERIOD_MS)
    {
        wl_send_data();
    }
    else { wl_send_image_slice(); }
    if (wifi_link_io_busy()) { wl_io_service(); }
}

void wifi_link_init(void)
{
    wl_time_init();
    wifi_link_io_init(wl_command_bytes, wl_tick_per_us);
    wl_state = WL_ST_BOOT;
    wl_seq = 0;
    wl_tx_fail = 0;
    wl_host_seen = 0;
    wl_socket_fail = 0;
    wl_c1_head = 0;
    wl_c1_tail = 0;
    wl_c0_len = 0;
    wl_cmd_len = 0;
    wl_cmd_discard = 0;
    wl_pending_kind = WL_TX_NONE;
    wl_next_host[0] = 0;
    wl_frame_crc_valid = 0;
    wl_idle_busy_ms = 0;
    wl_frame_pos = 0;
    wl_share.frame_want = 0;
    wl_share.frame_ready = 0;
    wl_share.frame_index = 0;
    wl_last_rx_ms = 0;
    wl_last_data_ms = 0;
    wl_last_log_ms = 0;
    wl_last_image_ms = 0;
    wl_frame_done_ms = 0;
    strcpy(wl_host, WIFI_LINK_HOST_IP);
    wifi_link_log("MCAR_APP built " __DATE__ " " __TIME__ "\r\n");
}

void wifi_link_service(void)
{
    wl_time_poll();

    switch (wl_state)
    {
        case WL_ST_BOOT:
        {
            if (wl_now_ms >= WIFI_LINK_BOOT_DELAY_MS)
            {
                wl_enter(WL_ST_INIT);
            }
        } break;

        case WL_ST_INIT:
        {
            uint8 result;
            wifi_link_io_init(wl_command_bytes, wl_tick_per_us);
            wl_pending_kind = WL_TX_NONE;
            wl_frame_pos = 0;
            result = wifi_spi_init(WIFI_LINK_SSID, WIFI_LINK_PASSWORD);
            if (0 == result)
            {
                wifi_link_log("[NET] module %s mac %s\r\n", wifi_spi_version, wifi_spi_mac_addr);
                wl_enter(WL_ST_SOCKET);
            }
            else
            {
                wl_backoff(WL_ST_INIT, "module init failed");
            }
        } break;

        case WL_ST_SOCKET:
        {
            uint8 result = wifi_spi_socket_connect("UDP", wl_host, WIFI_LINK_HOST_PORT, WIFI_LINK_LOCAL_PORT);
            if (0 == result)
            {
                wifi_link_log("[NET] udp %s:%s local %s ip %s\r\n",
                              wl_host, WIFI_LINK_HOST_PORT, WIFI_LINK_LOCAL_PORT, wifi_spi_ip_addr_port);
                wl_host_seen = 0;
                wl_socket_fail = 0;
                wl_last_data_ms = wl_now_ms;
                wl_last_log_ms = wl_now_ms;
                wl_last_rx_ms = wl_now_ms;
                wl_last_image_ms = wl_now_ms;
                wl_frame_done_ms = wl_now_ms;
                wl_enter(WL_ST_RUN);
            }
            else
            {
                wl_socket_fail++;
                if (wl_socket_fail >= 6)
                {
                    wl_socket_fail = 0;
                    wl_backoff(WL_ST_INIT, "socket retries exhausted");
                }
                else
                {
                    wl_backoff(WL_ST_SOCKET, "socket connect failed");
                }
            }
        } break;

        case WL_ST_RUN:
        {
            wl_run();
        } break;

        case WL_ST_BACKOFF:
        {
            if ((wl_now_ms - wl_backoff_ms) >= WIFI_LINK_RETRY_MS)
            {
                wl_enter(wl_backoff_next);
            }
        } break;

        default:
        {
            wl_enter(WL_ST_BOOT);
        } break;
    }

    system_delay_us(300);           // 让出 CPU0，同时给控制中断留出响应时间
}

//=====================================================CPU1 侧=====================================================
void wifi_link_cpu1_service(uint8 new_frame)
{
    static uint32 last_ms;
    static uint32 last_frame_ms;
    static uint8 have_sample;
    uint32 now = system_count;
    float ax, ay, az, gz;
    wl_share.sequence++;
    __dsync();
    gz = imu660rb_gyro_transition(icm_data_lpf.gyroZ);
    wl_share.gz = gz;
    wl_share.gx = imu660rb_gyro_transition(icm_data_lpf.gyroX);
    wl_share.gy = imu660rb_gyro_transition(icm_data_lpf.gyroY);
    ax = imu660rb_acc_transition(icm_data_lpf.accX);
    ay = imu660rb_acc_transition(icm_data_lpf.accY);
    az = imu660rb_acc_transition(icm_data_lpf.accZ);
    wl_share.ax = ax; wl_share.ay = ay; wl_share.az = az;
    /* Sensor-frame tilt in degrees. Control filter values are raw counts,
       so they must never be presented as physical Euler angles. */
    if (ax * ax + ay * ay + az * az > 0.000001f)
    {
        wl_share.roll_deg = atan2f(ay, az) * 57.2957795f;
        wl_share.pitch_deg = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.2957795f;
    }
    if (have_sample)
    {
        wl_share.yaw_deg = fmodf(wl_share.yaw_deg + gz * (float)(uint32)(now - last_ms) * 0.001f, 360.0f);
    }
    have_sample = 1;
    last_ms = now;
    wl_share.erro_px = erro;
    /* Yaw error loop output is a raw gyro target. Inner-loop output is PWM. */
    wl_share.omega_cmd = -balance_cascade.yaw_erro_cycle.out * imu660rb_gyro_transition(1);
    wl_share.vx_cmd = expect_speed * (50.0f * WIFI_LINK_CM_PER_COUNT);
    wl_share.vision_event = (beacon.sum > 0 && beacon.centerX >= 10 &&
                            beacon.centerX < WIFI_LINK_IMG_W && beacon.centerY >= 39 &&
                            beacon.centerY < WIFI_LINK_IMG_H - 10) ? 1u : 0u;
    if (new_frame)
    {
        if (wl_share.vision_frames) { wl_share.vision_interval_us = (uint32)(now - last_frame_ms) * 1000u; }
        last_frame_ms = now;
        wl_share.vision_frames++;
    }
    __dsync();
    wl_share.sequence++;
    __dsync();
    wl_share.cpu1_ready = 1;
    if (new_frame && wl_share.frame_want && !wl_share.frame_ready)
    {
        memcpy(wl_frame, image_binary, WIFI_LINK_IMG_SIZE);
        wl_share.frame_index++;
        __dsync();
        wl_share.frame_want = 0;
        wl_share.frame_ready = 1;
    }
}
