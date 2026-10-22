/*
 * wifi_link.h
 *
 * WiFi SPI 无线链路（移植自 McarPVC / CYT4BB7 的 wifispi 方案）
 *
 * 协议与上位机 WiFiSPI_Monitor（MCAR/MCAM）保持一致：
 *   上行  单端口 6666 -> 电脑 8081
 *         MUDP 头(8B): 'M''U''D''P' + 版本1 + 类型(1参数/2日志/3图像) + 负载长度(u16 LE)
 *         类型1: 148 字节参数包
 *         类型2: UTF-8 文本日志
 *         类型3: MCAM 24 字节头 + 1400 字节分片
 *   下行  "MCAR ALIVE <id>" / "MCAR GET <id> ALL" / "MCAR SET <id> <key> <value>"
 *
 * 运行位置：
 *   CPU0 空主循环  wifi_link_service()       —— SPI 事务、状态机、发包
 *   CPU1 视觉主循环 wifi_link_cpu1_service()  —— 只做内存拷贝/发布遥测，不碰 SPI
 *   控制 1ms 中断 直接读 wifi_target_speed / 累加 wifi_encoder_total
 */

#ifndef CODE_WIFI_LINK_H_
#define CODE_WIFI_LINK_H_

#include "zf_common_headfile.h"

//=====================================================网络参数=====================================================
// Copy wifi_link_config.example.h to wifi_link_config.h and configure locally.
#include "wifi_link_config.h"
// 车模与电脑必须连接同一个 WiFi；本机配置文件不提交到 Git。
// 也可在运行中通过 "MCAR SET <id> host <a.b.c.d>" 修改（见 wifi_link.c）

#include "config.h"

//=====================================================对外变量=====================================================
extern volatile float wifi_target_speed;                             // 前进目标（编码器计数/20ms）
extern volatile float wifi_ramp_step;                                // 起步斜坡，每 1ms 增量
extern volatile float wifi_ramp_ms;                                  // 上位机 ramp_ms 回读值
extern volatile float wifi_deadband_px;                              // 视觉死区，0 表示关闭
float wifi_vision_erro(float erro_value);                   // 死区内返回 0
extern volatile uint32 wifi_encoder_total;                // 行进轮编码器累计计数

//=====================================================对外接口=====================================================
void  wifi_link_init(void);                     // CPU0 上电调用一次（不阻塞）
void  wifi_link_service(void);                  // RUN 分步；INIT/SOCKET 仍调用阻塞 SDK
void  wifi_link_cpu1_service(uint8 new_frame);             // CPU1 采样/拷贝，不进行 SPI 通信
uint8 wifi_link_online(void);                   // 1 = WiFi 已连接并处于收发状态

void  wifi_link_log(const char *format, ...);   // CPU0 侧日志（参数应答等）
void  wifi_link_cpu1_log(const char *text);     // CPU1 侧日志（单行，自动补 \r\n）

#endif /* CODE_WIFI_LINK_H_ */
