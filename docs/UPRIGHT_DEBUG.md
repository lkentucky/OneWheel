# 独轮直立调试说明

> **安全输出更新：** 调试前请读 [SAFETY_FIXES_20261022.md](SAFETY_FIXES_20261022.md)；硬件输出改由CPU1独立1ms ISR负责，启动增加完成计算/反馈新鲜度门槛，新增故障锁存需复位清除。

当前版本以静止直立调试为默认目标：上电停车，目标速度为 0，保留 IMU、编码器、行进轮、动量轮和 WiFi 遥测。摄像头初始化、图像处理、IPS200 显示、WiFi 图传均默认关闭。代码已编译并通过主机测试，尚未烧录或验证实车能够直立；原车 PID 和机械零点仍须按本车标定。

## 主循环与执行位置

`user/cpu0_main.c` 只调用 `app_cpu0_init()` 和 `app_cpu0_poll()`。CPU0 初始化时钟与调试串口，轮询 WiFi 状态机，处理收发和参数命令。

`user/cpu1_main.c` 只调用 `app_init()` 和 `app_poll()`。CPU1 初始化电机零输出、UART3 小驱动、控制参数和 IMU，随后扫描按键并发布遥测：

```c
void app_poll(void)
{
    app_new_frame = 0;
    app_buttons_poll();
    // app_camera_poll();
    // app_camera_display_poll();
    app_telemetry_poll();
    system_delay_ms(APP_MAIN_LOOP_DELAY_MS);
}
```

实际平衡运算在 `user/isr.c → CCU60_CH0_ISR → pit_isr_callback()`，该中断配置在 **CPU0**。CPU1 调用 `pit_ms_init()` 不改变中断所属核。

| 周期 | 工作 | 停车时 |
| --- | --- | --- |
| 1 ms | IMU 采样、低通、角速度内环、保护与电机输出 | 仍采样；不积分 PID，不驱动 |
| 5 ms | 两轴互补滤波、角度环 | 仍更新滤波，便于观察零点 |
| 20 ms | 行进轮编码器读取清零、动量轮速度请求、速度环 | 仍读取反馈 |
| 10 ms | CPU1 按键扫描、共享遥测发布 | 继续执行 |
| 约 50 ms | CPU0 UDP 数据发送 | 继续执行；实际周期受 SPI 忙与网络影响 |

## 启停和模块选择

上电后所有电机保持零输出。IMU 初始化成功并经过 2 秒预热后，按车载 **K1** 请求启动。K2/K3/K4 或上位机停车按钮请求停止；停止优先于同时发生的启动请求。保护触发时本次控制回调就停止所有输出，并清除 PID 历史，保留已设置的增益和零点。网络不提供启动。

目标速度 0 表示原地平衡目标，**仍可能输出电机指令**，不能代替停车。上位机停车回包表示车端已接收请求；再观察新遥测的停车状态和三路 duty=0。

应用参数集中在 `Seekfree_TC264_Opensource_Library/code/config.h`。修改后重新编译烧录。WiFi 凭据单独保存在被 Git 忽略的 `wifi_link_config.h`，避免上传私人网络信息；SDK 内部寄存器常量不移入应用配置。

| 用途 | 配置 |
| --- | --- |
| 当前默认 | DRIVE=1，ROLL=1，PITCH=1，MOTOR=1，YAW=0；四个摄像头/屏幕相关 APP 开关为 0 |
| 只看传感器 | `APP_DRIVE_ENABLE=0`；仍有传感器/编码器/遥测，K1 不允许启动 |
| 只调动量轮平衡 | DRIVE=1，`CONTROL_ROLL_ENABLE=1`，PITCH=0，MOTOR=0，YAW=0 |
| 只调行进轮 | DRIVE=1，ROLL=0，PITCH=1，MOTOR=1，YAW=0 |

其中 DRIVE 指 `APP_DRIVE_ENABLE`；ROLL/PITCH/MOTOR/YAW 指 `CONTROL_*_ENABLE`。ROLL 控制动量轮平衡，PITCH 控制行进轮俯仰，MOTOR 是额外叠加的行进轮速度 PID。PITCH 和 MOTOR 都关闭时，行进轮禁止输出。YAW 当前关闭；摄像头关闭时视觉误差不会参与控制。

## 配置与标定重点

`config.h` 按应用开关、周期、速度与方向、IMU、保护、PID、视觉、显示、WiFi/硬件引脚分组。主要待标定项：

1. **IMU 安装轴与方向**：当前动量轮轴取 gyroY/accZ，行进轴取 gyroZ/accY，转向取 gyroX；确认实际安装后再调控制增益。
2. **静态陀螺零偏**：`IMU_GYRO_BIAS_X/Y/Z` 是原始计数，采样后减去；`IMU_GYRO_DEADBAND_RAW` 和低通时间常数也可调整。当前值沿用原车。
3. **机械平衡零点**：`ROLL_MECHANICAL_ZERO` 和 `PITCH_MECHANICAL_ZERO` 加在互补滤波内部量上，单位是原始内部量，不是度。停车时可用上位机 `roll_zero/pitch_zero` 临时调整。滤波结果约等于 `angle_temp + mechanical_zero`；希望当前位置作为零点时，用当前滤波读数反推需要减少的零点偏移。
4. **反馈与输出方向**：行进轮 `ENCODER_DIRECTION_SIGN/MOTOR_DIRECTION_SIGN`，动量轮 `MOMENTUM_LEFT_SIGN/RIGHT_SIGN`。确认反馈变化方向与实际转向相符。
5. **编码器比例**：行进轮使用 GPT12 T2，P33.7 为脉冲、P33.6 为方向（按本车实际接线修正），单边沿脉冲加方向模式。软件不能确定编码器物理型号、线数或每圈计数。动量轮速度由 UART3 驱动板回传，单位沿用驱动。`WIFI_LINK_CM_PER_COUNT` 默认 0，未标定时不提供 cm/s。
6. **电机与驱动**：有刷中轮为PWM＋DIR，P21.5输出17 kHz PWM、P21.4输出方向电平；正指令DIR=0，负指令DIR=1。GND共地，三个NC悬空。切换方向前先清零PWM命令，停车/保护时PWM为0；UART3 为 460800，TX=P15.6/RX=P15.7。`MOTOR_DEADZONE_DUTY`、输出限幅须按驱动/电机确定。
7. **PID**：9 组 P/I/D、积分限幅、积分系数和输出限幅均在配置中。先区分角速度、角度、速度环与额外 MOTOR 环的作用；本版本同时修正旧增量 PID 的 P/D 历史差分及小数积分丢失，旧车增益不能当成已验证整定结果。
8. **保护**：`PROTECT_ROLL_RAW/PITCH_RAW` 是原始内部量阈值，不是倾角度数；动量轮/行进轮限幅也在配置内。

当前 PID 路径：动量轮速度差 → roll_speed → 原始平衡目标 → roll_angle → 原始角速度目标 → roll_rate → 左右动量轮；行进轮编码器 → pitch_speed → 俯仰目标 → pitch_angle → 取负后 pitch_rate，再叠加 motor 环输出给行进轮。转向环当前不参与输出。

## 上位机

已同步修改 `C:\Users\Lenovo\Desktop\麦轮资料\McarPVC\upper_computer`，新增独轮配置；麦轮模式仍是 `app.py` 的默认模式。独轮程序：

桌面入口已替换为 `C:\Users\Lenovo\Desktop\WiFiSPI_Monitor.exe`，直接双击即可进入独轮模式。旁边的 `WiFiSPI_Monitor_Runtime` 是运行依赖，请保留。旧桌面程序备份在上位机源码的 `backups/desktop_20261021_195916/`。

便携原件为 `upper_computer/release/OneWheelWiFiMonitor_Portable/OneWheelWiFiMonitor_Portable.exe`。移动时保留整个便携目录及 `WiFiSPI_Monitor_Runtime`；不依赖系统Python，也无需在启动时解包。另保留单文件 `release/OneWheelWiFiMonitor.exe`，其启动需要可写临时目录。

源码启动可使用本仓库入口，自动读取当前固件 `config.h`，并将采集/设置保存到 `.monitor_data/`：

```powershell
& 'C:\Users\Lenovo\Desktop\麦轮资料\WiFiSPI_Monitor\.venv\Scripts\python.exe' tools/monitor_onewheel.py
```

独立 EXE 使用打包时的默认参数快照。更改固件配置后，可以运行 `OneWheelWiFiMonitor.exe --firmware-config "实际路径/config.h"`，或用 `tools/build_onewheel.ps1 -FirmwareConfig "实际路径/config.h"` 重新打包。不要直接修改生成的 `onewheel_config.h` 快照。

1. 电脑与模块网络互通，车端 WiFi 配置指向电脑 IP；程序选择单端口 MUDP、监听 8081，点击开始监听。
2. 调参目标设为模块 IP、端口 6666；“读取车辆全部参数”完整确认后，显示 34 项可调值及运行状态。
3. 参数页预设“独轮完整遥测 / 直立控制量 / 电机与速度”，按原生含义命名；曲线页可选择需要观察的量。
4. 直立相关量：`roll_balance_raw/pitch_balance_raw`、相应 target、原始 gyro feedback、各内环输出、`motor_output`、三路实际 duty、两项机械零点和 `control_flags`。roll/pitch/yaw 角度仍用于姿态观察，不能代替控制内部量。
5. 34 项可调值为 speed、k1/k2、roll_zero/pitch_zero、9 组 PID 的 P/I/D、ramp_ms、deadband_px。PID、零点、视觉系数和斜坡只允许停车修改；speed 可在线修改。启动只允许 K1。
6. 方案导入、默认值填入和本地保存均不会自动修改车辆；逐项应用并收到匹配 ID 的车辆确认后才算成功。参数仅保存在 RAM，复位恢复 config.h。

数据过期 1 秒后，独轮状态明确显示未知。独轮模式不发送麦轮固件的 ALIVE 命令。摄像头页保留供以后恢复使用，当前没有图像属于正常行为。

## 恢复摄像头与 IPS200

设置 `APP_CAMERA_ENABLE=1`，取消 `app_poll()` 中 `app_camera_poll()` 的注释。若需屏幕图像，再设置 `APP_IPS200_ENABLE=1`、`APP_CAMERA_DISPLAY_ENABLE=1`，取消 `app_camera_display_poll()` 注释。若需 WiFi 图传，设置 `APP_WIFI_IMAGE_ENABLE=1`。显示接口/引脚、图像阈值、标记尺寸等均从 config.h 读取。

## 已完成验证与边界

2026-10-21：ADS 1.10.10 / TASKING 1.1r8 全量构建 168 个目标文件，0 errors、0 warnings，生成 ELF/HEX/MAP。原生主机测试实际编译 app/PID/control/motor/vision/WiFi 源码，覆盖默认、仅观察、仅动量轮、仅行进轮及行进方向取反五种配置，以及启动门槛、停车优先、保护立即归零、参数边界和数据包。

上位机单元测试和真实 GUI 的 UDP 回环测试另外执行，使用实际 C 测试产生的遥测与参数回包。硬件 SPI、IMU、电机和 UART 在这些测试中使用替身；没有烧录、实车闭环、无线吞吐或中断抖动测量。

本次上位机165项测试全部通过；源码GUI完成35项参数回读和停车确认。便携版及替换后的桌面程序均在默认环境中完成启动与真实UDP回环收包，解析错误为0。单文件版在测试指定的可写临时目录下通过同类验证。
