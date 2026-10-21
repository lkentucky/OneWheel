# WiFi-SPI UDP 传输

传输层移植自 `MCARpreparation/project/code/wifispi.c/.h`，底层继续使用 Qcar 的逐飞 TC264 WiFi-SPI 驱动。需要 WiFi-SPI 2.0 模块，固件版本以 `V2` 开头。

## 连接

1. 核对 `code/qcar_config.h` 中的 `QCAR_WIFI_SSID`、`PASSWORD`、`TARGET_IP`、`TARGET_PORT`、`LOCAL_PORT`。默认沿用 MCARpreparation 当前配置；PC 接收端口为 8081，模块端口为 5001。目标 IP 应为当前 PC 的局域网 IP。
2. SPI3 接线：SCK=P22.3、MOSI=P22.0、MISO=P22.1、CS=P22.2、INT=P15.8、RST=P23.1，另接模块要求的电源和共地。引脚和 30 MHz SPI 速率由 `libraries/zf_device/zf_device_wifi_spi.h` 配置。
3. PC 打开原 MCAR 上位机的 UDP/JustFloat 接收功能，监听上述 PC 端口。先执行 `LIST?` 刷新 Qcar 的变量名，再重新选择通道；MCAR 保存的麦轮、导航通道及滑块布局需要替换。
4. 车辆停止、Camera/IMU/Wheel test 预览关闭时，进入 **WiFi → Connect**，按 KEY_3 选中、KEY_1 设为 On。连接失败后再次按 KEY_1 可重试。初始化最多尝试 3 次，连接过程可能阻塞菜单。
5. 连接后默认发送 roll/pitch/yaw。使用 **Startup → IMU** 启动姿态预览与标定，或按原方式启动控制。仅连接 WiFi 不会启动 IMU 或电机。
6. 断开时先停止运行和预览，再将 Connect 设为 Of。Startup → Stop 保持遥测连接，便于查看停车状态。

UDP 回包始终发往配置中的 PC IP/端口。PC 下发命令应发往模块的 IP 和本地端口 5001。模块地址可在调试器中查看 `wifi_spi_ip_addr_port`。协议回复保留 `MCAR` 前缀，以兼容原上位机。

## 数据帧及命令

一帧为 N 个小端 IEEE-754 float32，后接 `00 00 80 7F`，每帧一个 UDP 数据报；N 范围为 1～40。默认 N=3，顺序为 `roll_deg,pitch_deg,yaw_deg`，长度 16 字节。文本回复与数据帧各自使用独立 UDP 数据报。

文本命令区分大小写，以 LF 或 CRLF 结束：

| 命令 | 作用 |
| --- | --- |
| `LIST?` | 回复所有可订阅变量名 |
| `GET?` | 回复当前通道、周期和发送开关 |
| `SUB roll_deg,pitch_deg,yaw_deg,gyro_z_dps` | 按给定顺序原子替换通道列表 |
| `RATE 20` | 设置发送最小间隔，允许 2～1000 ms |
| `STREAM 0` / `STREAM 1` | 暂停／恢复数据帧；命令接收一直工作 |

未知变量、重复通道、空字段及超长命令返回 `MCAR ERR ...`，保留上一份有效配置。单次最多读取并解析 256 字节，接收轮询最小间隔 10 ms。

滑块沿用原项目格式，可不带换行，支持分段接收、连续拼接及 JSON 引号：

```text
[slider,cascade_ang_roll.kp,6]
["slider","cascade_ang_vel_pitch.ki",0.5]
[slider,balance_angle_x,-1.25]
```

允许修改的 PID 名为 `cascade_vel_roll/pitch/yaw`、`cascade_ang_roll/pitch/yaw`、`cascade_ang_vel_roll/pitch/yaw`；每个对象可写 `.kp`、`.ki`、`.kd`，范围 0～1000。`balance_angle_x/y` 范围为 -45～45 度。收到后直接更新与本地菜单相同的变量，可在运行中调参；不主动清 PID 积分。越界、NaN/Inf、未知参数及格式错误均拒绝。下发只修改 RAM，需要停车后通过 Storage → Save 保存。

## 可订阅变量

| 类别 | 变量 |
| --- | --- |
| 欧拉角，度 | `roll_deg`, `pitch_deg`, `yaw_deg` |
| 原始传感器，g / 度每秒 | `accel_x_g`, `accel_y_g`, `accel_z_g`, `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` |
| 状态与标定 | `imu_status`, `imu_cal_percent`, `gz_bias_dps`, `accel_norm_g`, `imu_seq`, `imu_method`, `gyro_z_integral_deg` |
| 解算诊断 | `imu_dt_ms`, `imu_solve_us`, `imu_solve_max_us`, `imu_accel_used`, `ekf_nis`, `ekf_bias_x_dps`, `ekf_bias_y_dps` |
| 车轮速度 | `wheel_speed`（real_speed）、`target_speed`（mono_params.setspeed），保留原控制单位 |
| 级联输出 | `roll_speed_out`, `pitch_speed_out`, `yaw_speed_out`, `roll_angle_out`, `pitch_angle_out`, `yaw_angle_out`, `roll_gyro_out`, `pitch_gyro_out`, `yaw_gyro_out` |

所有通道转换为 float32。`imu_status` 等含义见 `code/imu.h`；`imu_seq` 等整数超过 2²⁴ 后，float32 不能逐整数精确表示。未启动 IMU 时角度为初始化值，应结合状态通道判断。

## 调用与并发

`code/wifispi.h` 保留原项目的 `vofa_pack()`、`wifispi_send_floats()`、`wifispi_send_imu()` 和 `wifi_justfloat(...)` 接口。CPU0 菜单调用 `wifispi_telemetry_init()` 连接、`wifispi_telemetry_service()` 轮询、`wifispi_telemetry_stop()` 断开。`wifispi_init()` 是底层连接入口，常规使用应调用完整的 telemetry 初始化接口。

所有 SPI 收发及文本解析在 CPU0 主循环开中断时执行。仅复制订阅数据和写入滑块变量时短暂关闭 CPU0 中断，保护同核姿态／控制中断的数据一致性。时间基准由 RT1064 DWT 改为 TC264 STM0，使用无符号差值处理计数回绕。

默认请求周期 10 ms；CPU0 菜单自身每轮有 10 ms 延时，并包含屏幕刷新及同步 WiFi 收发，因此实际发送周期取决于主循环耗时，`RATE 2` 不保证 500 Hz。模块失联后需停车并从 Connect 重试，不在控制运行期间自动重新初始化。

`QCAR_WIFI_ENABLED=1`（默认）时，CPU0 独占模块，CPU1 继续处理视觉。设为 0 后，UDP 模块不触碰 SPI/GPIO，可恢复原 CPU1 TCP 相机预览分支（`cam_flag=0`、`wifi_spi_flag=1`，TCP 地址仍取驱动头文件配置）。两种传输模式需要分别编译，不可同时启用。MCAR 的麦轮参数、GOTO/STOP/ROUTE 命令未接入 Qcar。

## 诊断与验证

调试器可查看 `imu_wifi_status`：0 未连接、1 就绪、2/3/4 为模块／热点／UDP 初始化阶段；-1 模块通信或版本不匹配、-2 热点失败、-3 socket 失败、-4 发送或断开失败、-5 编译禁用。`imu_wifi_last_error` 保留本轮初始化最近错误，后续重试成功不清零。

`imu_wifi_init_attempts` 为本轮尝试次数，`imu_wifi_tx_packets` 为累计成功遥测帧数；`wifi_telemetry_commands`、`wifi_telemetry_command_errors` 为命令统计。初始化失败可通过这些值定位到阶段。

运行 `python tests/check_wifi.py`，使用真实移植源文件与模拟 STM/SPI，检查字节序、帧边界、订阅、滑块、错误输入、发送失败、重试、断开、计数回绕及禁用模式无硬件依赖。板上 SPI 通信、UDP 联机和控制实时性仍需接硬件验证。
