# OneWheel

> **行进编码器接线修正：** 按本车实际接线改用 GPT12 T2：P33.7计数、P33.6方向。此前P20.3/P20.0配置与接线不符；需要重新编译并烧录后，停车手转轮子验证计数及正反方向。

> **安全输出修复已更新：** 当前电机运行期输出由CPU1独立1ms中断统一管理，启动增加控制心跳/飞轮反馈新鲜度检查，新增故障需复位清除。详见 [五项修复说明](docs/SAFETY_FIXES_20261022.md)。上位机协议与硬件引脚保持原样。

> **当前快照仍有待修复问题：** 第二轮复审已复现跨核时间采样导致误停、旧反馈在计时器完整回绕后再次有效，以及IMU缺少运行期健康检查。此版本保留现状供备份，具体触发条件、验证证据及参考无刷驱动问题见 [第二轮复审报告](docs/SAFETY_REVIEW_SECOND_20261022.md)。

基于 TC264D 的独轮信标车固件，整理自中国矿业大学第 20 届全国大学生智能汽车竞赛独轮信标项目，并使用逐飞 TC264 开源库。本仓库首版将原 TFT1.8 显示改为 **IPS200 2.0 英寸屏（SPI 接口）**，保留原车的控制与视觉算法。

## 当前默认：直立调试

2026-10-21：双核入口已简化为 `app_init()/app_poll()` 和 CPU0 通信轮询；应用开关、PID、零点、方向、周期、保护和引脚集中在 `code/config.h`。上电停车，预热2秒后按K1启动；K2/K3/K4或上位机停车。目标速度默认0，摄像头、IPS200图像和WiFi图传默认关闭，停止时仍可观察传感器/编码器。

上位机已同步增加独轮模式，提供三路电机反馈/输出、原始平衡量、PID输出、运行状态和34项原生调参。使用独立 `OneWheelWiFiMonitor.exe`，不覆盖麦轮程序。完整入口、参数标定、模块选择和恢复图像步骤见 [直立调试说明](docs/UPRIGHT_DEBUG.md)。

当前全量编译：ADS 1.10.10 / TASKING 1.1r8，169个目标文件，0错误、0警告。既有六配置主机回归通过；第二轮复审的定向复现另外确认了上述待修复问题。未烧录、未实车验证。

## 开发环境

- 单片机：Infineon AURIX TC264D / TC26x。
- IDE：**AURIX Development Studio（ADS）1.10.10**。
- 编译器：ADS 自带的 **TASKING v1.1r8**，使用工程的 `Debug` 配置。
- 底层：原项目逐飞 TC264 库及 Infineon iLLD；本次没有整体升级驱动库。

这里使用 ADS，与 AURIX Configuration Studio（ACS）及其 GCC 工具链不同。原项目按 TASKING 构建，不应直接切换成 GCC。

## 项目包含什么

- **视觉信标识别**：MT9V03X 图像采集、Otsu 阈值计算、二值化、候选信标搜索与偏差计算。启用视觉时阈值下限默认为180，可在config.h调整。
- **姿态和平衡控制**：IMU660RB 数据采集、滤波、速度/角度/角速度控制及偏航控制。
- **电机接口**：中轮采用 PWM＋DIR（P21.5输出17 kHz PWM、P21.4输出方向电平），编码器计数，以及UART3小驱动板通信。
- **屏幕调试（默认关闭）**：IPS200 可显示二值化图像、当前/历史信标标记和偏差值。首版保持原来的半尺寸图像和标记坐标，替换对应的显示驱动接口。
- **按键及串口调试**：K1请求启动、K2/K3/K4停车；无线串口CSV调试默认关闭。
- **WiFi SPI 上位机链路**：SPI_3 上的 WiFi 模块，单 UDP 端口复用（数据 / 日志 / 图像），
  兼容麦轮项目 `McarPVC` 的 WiFiSPI_Monitor：姿态、编码器、视觉偏差、188×120 灰度图像、
  日志镜像与在线调参。实现、协议、参数表与构建方式见 [WIFI_LINK.md](WIFI_LINK.md)。

CPU0主循环负责WiFi链路，CPU1主循环负责按键和遥测发布；摄像头处理和显示已封装为独立函数并在主循环注释。`user/isr.c` 的CPU0 CCU60中断进入1 ms控制回调，包含5 ms、20 ms分频与2 s预热门槛。

以上描述来自当前源码；具体电机型号、机械结构和驱动板型号不能仅由代码确定。

## 目录

```text
oneWheel/
├── README.md
├── .gitignore
└── Seekfree_TC264_Opensource_Library/
    ├── .project / .cproject / .settings/  ADS 工程配置
    ├── user/                            双核入口、中断
    ├── code/                            控制、视觉、电机与通信
    ├── libraries/                       逐飞驱动、iLLD、配置静态库
    └── Lcf_Tasking_Tricore_Tc.lsl        TASKING 链接脚本
```

`code/four_posture.c` 当前内容均为注释，`code/picture_process.c` 只有说明注释。两者保留作为历史/占位文件，已在工程构建配置中排除，避免 `W508 empty source file` 警告。以后加入有效实现时，需取消对应文件的 `Exclude from Build`。

## IPS200 屏幕接线

本次参考逐飞样例 `E8_14_mt9v03x_ips200_display_demo` 的 **`IPS200_TYPE_SPI`** 模式，调用项目已有的 `zf_device_ips200` 驱动。SPI/控制引脚与原 TFT1.8 配置一致：

| IPS200 信号 | TC264 引脚/配置 |
| --- | --- |
| SCL / SCK | P15.3，SPI2 SCLK |
| SDA / MOSI | P15.5，SPI2 MOSI |
| RST | P15.1 |
| DC | P15.0 |
| CS | P15.2 |
| BL | P15.4 |
| GND、供电 | 按屏幕模块及主板的实际接口规格连接 |

显示接口选择为 `code/config.h` 中的 `DISPLAY_INTERFACE`；屏幕引脚也集中在该文件，设备头文件通过别名读取。当前直立调试默认不初始化屏幕。如果实际屏幕是 8 位并口版，需同时调整接口模式、接线和对应引脚，不能直接沿用本表。

## 导入与编译

1. 安装 ADS 1.10.10，选择英文路径的工作空间。
2. 复制 `Seekfree_TC264_Opensource_Library/code/wifi_link_config.example.h` 为同目录的 `wifi_link_config.h`，填写 WiFi 和电脑 IP；本机配置已被 Git 忽略。
3. `File → Import → General → Existing Projects into Workspace`，选择仓库中的 `Seekfree_TC264_Opensource_Library`；建议复制到工作空间。
4. 将工程设为活动工程，选择 `Debug`，执行 `Clean Project → Build Project`。
5. 确认编译、链接成功，在工程 `Debug/` 下取得新生成的 ELF、HEX。

ADS 自带的 TASKING 工具要求通过 IDE 使用。单独在终端运行其编译器可能报 `F104: License does not support running as standalone`；使用 ADS 内的构建操作。

本仓库不提交生成的固件和构建缓存；从源码重新构建。2026-10-20，IPS200 修改后的首版已在 ADS 1.10.10 的 `Debug` 配置下完成独立副本的全量构建：**0 errors、0 warnings**，生成 165 个目标文件及新的 ELF、HEX、MAP。此次未烧录或上板验证。

此前WiFi SPI移植经独立审查修复，于2026-10-21完成构建（本次直立重构之前）：**0 errors、0 warnings**，167 个目标文件。RUN 采用分步 SPI 状态机；数据、日志和图像沿用现有上位机协议，独轮原生调参使用 `tools/monitor_onewheel.py`。实际 C 协议测试及上位机启动测试通过，尚未烧录或上板验证。接线、参数单位、启动方法和验证边界见 [WIFI_LINK.md](WIFI_LINK.md)，审查结论见 [DEEPSEEK_REVIEW.md](docs/DEEPSEEK_REVIEW.md)。

## `.metadata` 是什么

如果 `oneWheel` 同时被选为 ADS/Eclipse 工作空间，IDE 会在根目录生成 `.metadata/`，保存工程关联、索引、构建日志、窗口布局、调试会话和本机偏好。它不是车辆源码，也不是项目的 `.project/.cproject` 工程定义。

`.metadata/` 已加入 `.gitignore`，保留在本机，不上传。克隆仓库后选择自己的工作空间并导入工程即可；不需要复制别人的 `.metadata`。删除它会丢失工作空间状态，因此日常无需删除。

## 来源与许可

- 原车代码：[zui811/CUMT2025-9](https://github.com/zui811/CUMT2025-9)。
- IPS200 适配参考本地逐飞仓库的 `Example/Motherboard_Demo/Motherboard_Demo/E8_camera/E8_14_mt9v03x_ips200_display_demo`。
- 保留源文件内的作者、版权和许可声明。逐飞库文件声明 GPL-3.0-or-later，相关说明位于 `libraries/doc/GPL3_permission_statement.txt`；Infineon iLLD 等第三方文件遵循各自文件中的许可。

这是软件整理与屏幕适配版本。电机、摄像头、IMU、屏幕及实际车体参数仍需按自己的硬件核对和调试；构建成功不等于整车已完成上板验证。
